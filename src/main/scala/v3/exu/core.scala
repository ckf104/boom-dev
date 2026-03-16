//******************************************************************************
// Copyright (c) 2015 - 2019, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// RISC-V Processor Core
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
// BOOM has the following (conceptual) stages:
//   if0 - Instruction Fetch 0 (next-pc select)
//   if1 - Instruction Fetch 1 (I$ access)
//   if2 - Instruction Fetch 2 (instruction return)
//   if3 - Instruction Fetch 3 (enqueue to fetch buffer)
//   if4 - Instruction Fetch 4 (redirect from bpd)
//   dec - Decode
//   ren - Rename1
//   dis - Rename2/Dispatch
//   iss - Issue
//   rrd - Register Read
//   exe - Execute
//   mem - Memory
//   sxt - Sign-extend
//   wb  - Writeback
//   com - Commit

package boom.v3.exu

import java.nio.file.{Paths}

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.rocket.Instructions._
import freechips.rocketchip.tile.{TraceBundle}
import freechips.rocketchip.rocket.{Causes, PRV, TracedInstruction}
import freechips.rocketchip.util.{Str, UIntIsOneOf, CoreMonitorBundle, PlusArg}
import freechips.rocketchip.devices.tilelink.{PLICConsts, CLINTConsts}

import boom.v3.common._
import boom.v3.ifu.{GlobalHistory, HasBoomFrontendParameters}
import boom.v3.exu.FUConstants._
import boom.v3.exu.EventCounter
import boom.v3.util._

/**
 * Top level core object that connects the Frontend to the rest of the pipeline.
 */
class BoomCore()(implicit p: Parameters) extends BoomModule
  with HasBoomFrontendParameters // TODO: Don't add this trait
{
  val io = IO(new Bundle {
    val hartid = Input(UInt(hartIdLen.W))
    val interrupts = Input(new freechips.rocketchip.rocket.CoreInterrupts(false))
    val ifu = new boom.v3.ifu.BoomFrontendIO
    val ptw = Flipped(new freechips.rocketchip.rocket.DatapathPTWIO())
    val rocc = Flipped(new freechips.rocketchip.tile.RoCCCoreIO())
    val lsu = Flipped(new boom.v3.lsu.LSUCoreIO)
    val ptw_tlb = new freechips.rocketchip.rocket.TLBPTWIO()
    val trace = Output(new TraceBundle)
    val fcsr_rm = UInt(freechips.rocketchip.tile.FPConstants.RM_SZ.W)
  })

  io.ptw_tlb := DontCare
  io.ptw := DontCare
  io.ifu := DontCare

  //**********************************
  // construct all of the modules

  // Only holds integer-registerfile execution units.
  val exe_units = new boom.v3.exu.ExecutionUnits(fpu=false)
  val jmp_unit_idx = exe_units.jmp_unit_idx
  val jmp_unit = exe_units(jmp_unit_idx)

  // Meanwhile, the FP pipeline holds the FP issue window, FP regfile, and FP arithmetic units.
  var fp_pipeline: FpPipeline = null
  if (usingFPU) fp_pipeline = Module(new FpPipeline)

  // ********************************************************
  // Clear fp_pipeline before use
  if (usingFPU) {
    fp_pipeline.io.ll_wports := DontCare
    fp_pipeline.io.wb_valids := DontCare
    fp_pipeline.io.wb_pdsts  := DontCare
  }

  val numIrfWritePorts        = exe_units.numIrfWritePorts + memWidth
  val numLlIrfWritePorts      = exe_units.numLlIrfWritePorts
  val numIrfReadPorts         = exe_units.numIrfReadPorts

  val numFastWakeupPorts      = exe_units.count(_.bypassable)
  val numAlwaysBypassable     = exe_units.count(_.alwaysBypassable)

  val numIntIssueWakeupPorts  = numIrfWritePorts + numFastWakeupPorts - numAlwaysBypassable // + memWidth for ll_wb
  val numIntRenameWakeupPorts = numIntIssueWakeupPorts
  val numFpWakeupPorts        = if (usingFPU) fp_pipeline.io.wakeups.length else 0

  val decode_units     = for (w <- 0 until decodeWidth) yield { val d = Module(new DecodeUnit); d }
  val dec_brmask_logic = Module(new BranchMaskGenerationLogic(coreWidth))
  val rename_stage     = Module(new RenameStage(coreWidth, numIntPhysRegs, numIntRenameWakeupPorts, false))
  val fp_rename_stage  = if (usingFPU) Module(new RenameStage(coreWidth, numFpPhysRegs, numFpWakeupPorts, true)) else null
  val pred_rename_stage = Module(new PredRenameStage(coreWidth, ftqSz, 1))
  val rename_stages    = if (usingFPU) Seq(rename_stage, fp_rename_stage, pred_rename_stage) else Seq(rename_stage, pred_rename_stage)

  val mem_iss_unit     = Module(new IssueUnitCollapsing(memIssueParam, numIntIssueWakeupPorts))
  mem_iss_unit.suggestName("mem_issue_unit")
  val int_iss_unit     = Module(new IssueUnitCollapsing(intIssueParam, numIntIssueWakeupPorts))
  int_iss_unit.suggestName("int_issue_unit")

  val issue_units      = Seq(mem_iss_unit, int_iss_unit)
  val dispatcher       = Module(new BasicDispatcher)

  val iregfile         = Module(new RegisterFileSynthesizable(
                             numIntPhysRegs,
                             numIrfReadPorts,
                             numIrfWritePorts,
                             xLen,
                             Seq.fill(memWidth) {true} ++ exe_units.bypassable_write_port_mask)) // bypassable ll_wb
  val pregfile         = Module(new RegisterFileSynthesizable(
                            ftqSz,
                            exe_units.numIrfReaders,
                            1,
                            1,
                            Seq(true))) // The jmp unit is always bypassable
  pregfile.io := DontCare // Only use the IO if enableSFBOpt

  // wb arbiter for the 0th ll writeback
  // TODO: should this be a multi-arb?
  val ll_wbarb         = Module(new Arbiter(new ExeUnitResp(xLen), 1 +
                                                                   (if (usingFPU) 1 else 0) +
                                                                   (if (usingRoCC) 1 else 0)))
  val iregister_read   = Module(new RegisterRead(
                           issue_units.map(_.issueWidth).sum,
                           exe_units.withFilter(_.readsIrf).map(_.supportedFuncUnits).toSeq,
                           numIrfReadPorts,
                           exe_units.withFilter(_.readsIrf).map(x => 2).toSeq,
                           exe_units.numTotalBypassPorts,
                           jmp_unit.numBypassStages,
                           xLen))
  val rob              = Module(new Rob(
                           numIrfWritePorts + numFpWakeupPorts, // +memWidth for ll writebacks
                           numFpWakeupPorts))
  // Used to wakeup registers in rename and issue. ROB needs to listen to something else.
  val int_iss_wakeups  = Wire(Vec(numIntIssueWakeupPorts, Valid(new ExeUnitResp(xLen))))
  val int_ren_wakeups  = Wire(Vec(numIntRenameWakeupPorts, Valid(new ExeUnitResp(xLen))))
  val pred_wakeup  = Wire(Valid(new ExeUnitResp(1)))

  require (exe_units.length == issue_units.map(_.issueWidth).sum)

  //***********************************
  // Pipeline State Registers and Wires

  // Decode/Rename1 Stage
  val dec_valids = Wire(Vec(coreWidth, Bool()))  // are the decoded instruction valid? It may be held up though.
  val dec_uops   = Wire(Vec(coreWidth, new MicroOp()))
  val dec_fire   = Wire(Vec(coreWidth, Bool()))  // can the instruction fire beyond decode?
                                                    // (can still be stopped in ren or dis)
  io.ifu.dec_fire := dec_fire

  val dec_ready  = Wire(Bool())
  val dec_xcpts  = Wire(Vec(coreWidth, Bool()))
  val ren_stalls = Wire(Vec(coreWidth, Bool()))

  // Rename2/Dispatch stage
  val dis_valids = Wire(Vec(coreWidth, Bool()))
  val dis_uops   = Wire(Vec(coreWidth, new MicroOp))
  val dis_fire   = Wire(Vec(coreWidth, Bool()))
  val dis_ready  = Wire(Bool())

  // Issue Stage/Register Read
  val iss_valids = Wire(Vec(exe_units.numIrfReaders, Bool()))
  val iss_uops   = Wire(Vec(exe_units.numIrfReaders, new MicroOp()))
  val bypasses   = Wire(Vec(exe_units.numTotalBypassPorts, Valid(new ExeUnitResp(xLen))))
  val pred_bypasses = Wire(Vec(jmp_unit.numBypassStages, Valid(new ExeUnitResp(1))))
  require(jmp_unit.bypassable)

  // --------------------------------------
  // Dealing with branch resolutions

  // The individual branch resolutions from each ALU
  val brinfos = Reg(Vec(coreWidth, new BrResolutionInfo()))

  // "Merged" branch update info from all ALUs
  // brmask contains masks for rapidly clearing mispredicted instructions
  // brindices contains indices to reset pointers for allocated structures
  //           brindices is delayed a cycle
  val brupdate  = Wire(new BrUpdateInfo)
  val b1    = Wire(new BrUpdateMasks)
  val b2    = Reg(new BrResolutionInfo)

  brupdate.b1 := b1
  brupdate.b2 := b2

  for ((b, a) <- brinfos zip exe_units.alu_units) {
    b := a.io.brinfo
    b.valid := a.io.brinfo.valid && !rob.io.flush.valid
  }
  b1.resolve_mask := brinfos.map(x => x.valid << x.uop.br_tag).reduce(_|_)
  b1.mispredict_mask := brinfos.map(x => (x.valid && x.mispredict) << x.uop.br_tag).reduce(_|_)

  // Find the oldest mispredict and use it to update indices
  var mispredict_val = false.B
  var oldest_mispredict = brinfos(0)
  for (b <- brinfos) {
    val use_this_mispredict = !mispredict_val ||
    b.valid && b.mispredict && IsOlder(b.uop.rob_idx, oldest_mispredict.uop.rob_idx, rob.io.rob_head_idx)

    mispredict_val = mispredict_val || (b.valid && b.mispredict)
    oldest_mispredict = Mux(use_this_mispredict, b, oldest_mispredict)
  }

  // Caution!! 不能在 flush 信号拉高时禁止 b2.mispredict 生效，这样
  // 会导致 rob 那边 clear 了预测错误的分支指令后边的指令（被 clear 的
  // 指令不再参与 rob rollback），但是 renaming table 以及 free list
  // 没有恢复到分支指令对应的 snapshot 状态，导致 cpu 整个状态异常了
  // 因此，为了使 FTQ 能够区分 mispredict 和 rob flush，还是在前端加一个
  // 信号好了
  b2.mispredict  := mispredict_val // && !rob.io.flush.valid
  b2.cfi_type    := oldest_mispredict.cfi_type
  b2.taken       := oldest_mispredict.taken
  b2.pc_sel      := oldest_mispredict.pc_sel
  b2.uop         := UpdateBrMask(brupdate, oldest_mispredict.uop)
  b2.jalr_target := RegNext(jmp_unit.io.brinfo.jalr_target)
  b2.target_offset := oldest_mispredict.target_offset

  val oldest_mispredict_ftq_idx = oldest_mispredict.uop.ftq_idx


  assert (!((brupdate.b1.mispredict_mask =/= 0.U || brupdate.b2.mispredict)
    && rob.io.commit.rollback), "Can't have a mispredict during rollback.")

  io.ifu.brupdate := brupdate

  for (eu <- exe_units) {
    eu.io.brupdate := brupdate
  }

  if (usingFPU) {
    fp_pipeline.io.brupdate := brupdate
  }

  // Load/Store Unit & ExeUnits
  val mem_units = exe_units.memory_units
  val mem_resps = mem_units.map(_.io.ll_iresp)
  for (i <- 0 until memWidth) {
    mem_units(i).io.lsu_io <> io.lsu.exe(i)
  }

  //-------------------------------------------------------------
  // Uarch Hardware Performance Events (HPEs)

  val perfEvents = new freechips.rocketchip.rocket.EventSets(Seq(
    new freechips.rocketchip.rocket.EventSet((mask, hits) => (mask & hits).orR, Seq(
      ("exception", () => rob.io.com_xcpt.valid),
      ("nop",       () => false.B),
      ("nop",       () => false.B),
      ("nop",       () => false.B))),

    new freechips.rocketchip.rocket.EventSet((mask, hits) => (mask & hits).orR, Seq(
//      ("I$ blocked",                        () => icache_blocked),
      ("nop",                               () => false.B),
      ("branch misprediction",              () => b2.mispredict),
      ("control-flow target misprediction", () => b2.mispredict &&
                                                  b2.cfi_type === CFI_JALR),
      ("flush",                             () => rob.io.flush.valid),
      ("branch resolved",                   () => b2.valid)
    )),

    new freechips.rocketchip.rocket.EventSet((mask, hits) => (mask & hits).orR, Seq(
      ("I$ miss",     () => io.ifu.perf.acquire),
      ("D$ miss",     () => io.lsu.perf.acquire),
      ("D$ release",  () => io.lsu.perf.release),
      ("ITLB miss",   () => io.ifu.perf.tlbMiss),
      ("DTLB miss",   () => io.lsu.perf.tlbMiss),
      ("L2 TLB miss", () => io.ptw.perf.l2miss)))))
  val csr = Module(new freechips.rocketchip.rocket.CSRFile(perfEvents, boomParams.customCSRs.decls))
  csr.io.inst foreach { c => c := DontCare }
  csr.io.rocc_interrupt := io.rocc.interrupt
  csr.io.mhtinst_read_pseudo := false.B

  val custom_csrs = Wire(new BoomCustomCSRs)
  custom_csrs.csrs.foreach { c => c.stall := false.B; c.set := false.B; c.sdata := DontCare }

  (custom_csrs.csrs zip csr.io.customCSRs).map { case (lhs, rhs) => lhs <> rhs }

  //val icache_blocked = !(io.ifu.fetchpacket.valid || RegNext(io.ifu.fetchpacket.valid))
  val icache_blocked = false.B
  csr.io.counters foreach { c => c.inc := RegNext(perfEvents.evaluate(c.eventSel)) }

  //****************************************
  // Time Stamp Counter & Retired Instruction Counter
  // (only used for printf and vcd dumps - the actual counters are in the CSRFile)
  val debug_tsc_reg = RegInit(0.U(xLen.W))
  val debug_irt_reg = RegInit(0.U(xLen.W))
  val debug_brs     = Reg(Vec(4, UInt(xLen.W)))
  val debug_jals    = Reg(Vec(4, UInt(xLen.W)))
  val debug_jalrs   = Reg(Vec(4, UInt(xLen.W)))

  for (j <- 0 until 4) {
    debug_brs(j) := debug_brs(j) + PopCount(VecInit((0 until coreWidth) map {i =>
      rob.io.commit.arch_valids(i) &&
      (rob.io.commit.uops(i).debug_fsrc === j.U) &&
      rob.io.commit.uops(i).is_br
    }))
    debug_jals(j) := debug_jals(j) + PopCount(VecInit((0 until coreWidth) map {i =>
      rob.io.commit.arch_valids(i) &&
      (rob.io.commit.uops(i).debug_fsrc === j.U) &&
      rob.io.commit.uops(i).is_jal
    }))
    debug_jalrs(j) := debug_jalrs(j) + PopCount(VecInit((0 until coreWidth) map {i =>
      rob.io.commit.arch_valids(i) &&
      (rob.io.commit.uops(i).debug_fsrc === j.U) &&
      rob.io.commit.uops(i).is_jalr
    }))
  }

  dontTouch(debug_brs)
  dontTouch(debug_jals)
  dontTouch(debug_jalrs)

  debug_tsc_reg := debug_tsc_reg + 1.U
  debug_irt_reg := debug_irt_reg + PopCount(rob.io.commit.arch_valids.asUInt)
  dontTouch(debug_tsc_reg)
  dontTouch(debug_irt_reg)

  //****************************************
  // Print-out information about the machine

  val issStr =
    if (enableAgePriorityIssue) " (Age-based Priority)"
    else " (Unordered Priority)"

  // val btbStr =
  //   if (enableBTB) ("" + boomParams.btb.nSets * boomParams.btb.nWays + " entries (" + boomParams.btb.nSets + " x " + boomParams.btb.nWays + " ways)")
  //   else 0
  val btbStr = ""

  val fpPipelineStr =
    if (usingFPU) fp_pipeline.toString
    else ""

  override def toString: String =
    (BoomCoreStringPrefix("====Overall Core Params====") + "\n"
    + exe_units.toString + "\n"
    + fpPipelineStr + "\n"
    + rob.toString + "\n"
    + BoomCoreStringPrefix(
        "===Other Core Params===",
        "Fetch Width           : " + fetchWidth,
        "Decode Width          : " + coreWidth,
        "Issue Width           : " + issueParams.map(_.issueWidth).sum,
        "ROB Size              : " + numRobEntries,
        "Issue Window Size     : " + issueParams.map(_.numEntries) + issStr,
        "Load/Store Unit Size  : " + numLdqEntries + "/" + numStqEntries,
        "Num Int Phys Registers: " + numIntPhysRegs,
        "Num FP  Phys Registers: " + numFpPhysRegs,
        "Max Branch Count      : " + maxBrCount)
    + iregfile.toString + "\n"
    + BoomCoreStringPrefix(
        "Num Slow Wakeup Ports : " + numIrfWritePorts,
        "Num Fast Wakeup Ports : " + exe_units.count(_.bypassable),
        "Num Bypass Ports      : " + exe_units.numTotalBypassPorts) + "\n"
    + BoomCoreStringPrefix(
        "DCache Ways           : " + dcacheParams.nWays,
        "DCache Sets           : " + dcacheParams.nSets,
        "DCache nMSHRs         : " + dcacheParams.nMSHRs,
        "ICache Ways           : " + icacheParams.nWays,
        "ICache Sets           : " + icacheParams.nSets,
        "D-TLB Ways            : " + dcacheParams.nTLBWays,
        "I-TLB Ways            : " + icacheParams.nTLBWays,
        "Paddr Bits            : " + paddrBits,
        "Vaddr Bits            : " + vaddrBits) + "\n"
    + BoomCoreStringPrefix(
        "Using FPU Unit?       : " + usingFPU.toString,
        "Using FDivSqrt?       : " + usingFDivSqrt.toString,
        "Using VM?             : " + usingVM.toString) + "\n")
  
  //-------------------------------------------------------------
  //Enable_Sample_Support: some special registers
  val procTag         = RegInit(0.U(32.W))
  val exitFuncAddr    = RegInit(0.U(vaddrBitsExtended.W))
  val warmupInstNum   = RegInit(0.U(64.W))
  val maxEventNum     = RegInit(0.U(64.W))
  val sampleEventSel  = RegInit(0.U(3.W))
  val pfc_maxPriv     = RegInit(0.U(2.W))
  val pfc_enable      = RegInit(0.U(1.W))
  val sampleHappen    = RegInit(0.U(32.W))
  val userExitPC      = RegInit(0.U(64.W))

  val nowWarmupInsts  = RegInit(0.U(64.W))
  val nowEventNum     = RegInit(0.U(64.W))

  val tempReg1  = RegInit(0.U(64.W))
  val tempReg2  = RegInit(0.U(64.W))
  val tempReg3  = RegInit(0.U(64.W))
  val tempReg4  = RegInit(0.U(64.W))

  val isTargetProc = procTag === 0x1234567.U
  val startCounter = csr.io.status.prv <= pfc_maxPriv && isTargetProc && (pfc_enable === 1.U)
  val isUserMode      = csr.io.status.prv === 0.U && RegNext(csr.io.status.prv === 0.U) && RegNext(RegNext(csr.io.status.prv === 0.U))
  val sampleValid = Wire(Bool())
  val warmupValid = Wire(Bool())
  if (IN_SIMULATION) {
    sampleValid := isTargetProc && maxEventNum =/= 0.U && warmupInstNum === 0.U
    warmupValid := isTargetProc && warmupInstNum =/= 0.U
  } else {
    sampleValid := isUserMode && isTargetProc && maxEventNum =/= 0.U && warmupInstNum === 0.U 
    warmupValid := isUserMode && isTargetProc && warmupInstNum =/= 0.U 
  }
  val overflow_event  = sampleValid &&  (nowEventNum > maxEventNum)
  

  //-------------------------------------------------------------
  //Enable_PerfCounter_Support
  val event_counters = Module(new EventCounter(exe_units.numIrfReaders))

  //reset event counters
  event_counters.io.reset_counter := false.B
  for (w <- 0 until coreWidth) {
    val uop = rob.io.commit.uops(w)
    when (rob.io.commit.valids(w) && uop.ucsrInst && uop.inst(31, 20) === SpecialInst_RstPFC) { //tag == 1024, reset all counters
      event_counters.io.reset_counter := true.B
    }
  }


  //reset counter when running > start
  when (warmupValid) {
    nowWarmupInsts := nowWarmupInsts + RegNext(PopCount(rob.io.commit.arch_valids.asUInt))
    when (nowWarmupInsts > warmupInstNum) {
      warmupInstNum := 0.U
      nowWarmupInsts := 0.U
      event_counters.io.reset_counter := true.B
      nowEventNum := 0.U
    }
  }

  //start read counter
  for (w <- 0 until exe_units.numIrfReaders) {
    event_counters.io.read_addr(w).valid := iss_valids(w) && iss_uops(w).readCounter && iss_uops(w).ldst =/= 0.U
    event_counters.io.read_addr(w).bits := iss_uops(w).inst(26, 20)
  }

  //update nowWarmupInsts
  when (sampleValid) { //usemode
    switch (sampleEventSel){
      is (0.U)  { nowEventNum := nowEventNum + RegNext(PopCount(rob.io.commit.arch_valids.asUInt)) }
      is (1.U)  {  }
      is (2.U)  {  }
      is (3.U)  {  }
    }
    // printf("wordValid, instnum: %d, eventnum: %d, maxnum: %d\n", nowWarmupInsts, nowEventNum, maxEventNum)
  }

  //connect signal to counters
  for (w <- 0 until subECounterNum*16) {
    event_counters.io.event_signals(w) := 0.U
  }

  // 统计 commit 指令中分支指令的占比
  val com_is_br   = Wire(Vec(coreWidth, Bool()))
  val com_is_jal  = Wire(Vec(coreWidth, Bool()))
  val com_is_jalr = Wire(Vec(coreWidth, Bool()))
  val com_is_ret  = Wire(Vec(coreWidth, Bool()))
  val com_is_call  = Wire(Vec(coreWidth, Bool()))

  // 统计预测错误的指令数目 
  val com_misp_br   = Wire(Vec(coreWidth, Bool()))
  val com_misp_jalr = Wire(Vec(coreWidth, Bool()))
  val com_misp_ret  = Wire(Vec(coreWidth, Bool()))

  // 统计提交的 fetch packet 造成的流水线气泡数目
  val com_ft_bsrc1 = io.ifu.ft_tsrc.valid && io.ifu.ft_tsrc.bits === BSRC_1
  val com_ft_bsrc2 = io.ifu.ft_tsrc.valid && io.ifu.ft_tsrc.bits === BSRC_2
  val com_ft_bsrc3 = io.ifu.ft_tsrc.valid && io.ifu.ft_tsrc.bits === BSRC_3
  val com_ft_bsrcc = io.ifu.ft_tsrc.valid && io.ifu.ft_tsrc.bits === BSRC_C

  // ROB flushes caused by MINI_EXCEPTION_MEM_ORDERING
  val rob_mem_ordering_flush = rob.io.mem_ordering_flush

  // Decode valid pattern statistics
  val dec_all_false = !dec_valids.reduce(_||_)
  val dec_all_true  = dec_valids.reduce(_&&_)

  // Frontend bubble statistics from frontend
  val f2_clear_bubble        = io.ifu.f2_clear_bubble
  val f3_clear_bubble        = io.ifu.f3_clear_bubble
  val predecode_clear_bubble = io.ifu.predecode_clear_bubble
  val rob_flush_bubble       = io.ifu.rob_flush_bubble
  val mispred_flush_bubble   = io.ifu.mispred_flush_bubble

  // Fetch buffer enqueue statistics from frontend
  val fb_enq_cnt  = io.ifu.fb_enq_cnt
  val fb_enq_fire = io.ifu.fb_enq_valid

  // Frontend s2 replay statistics from frontend
  val s2_replay_total    = io.ifu.s2_replay_total
  val s2_replay_itlb_miss = io.ifu.s2_replay_itlb_miss
  val s2_replay_ic_miss   = io.ifu.s2_replay_ic_miss

  // BPD-ahead-of-IFU distance bucket from frontend (0-6)
  val bpd_ifu_dist_bucket = io.ifu.bpd_ifu_dist_bucket

  // Prefetch refill distance bucket from frontend
  val pf_refill_dist_bucket = io.ifu.pf_refill_dist_bucket

  // Frontend s0 stall statistics from frontend
  val s0_ifu_real_not_valid = io.ifu.s0_ifu_real_not_valid
  val s0_ifu_ftq_backpress  = io.ifu.s0_ifu_ftq_backpress

  for(w <- 0 until coreWidth) {
    val uop = rob.io.commit.uops(w)
    val valid = rob.io.commit.arch_valids(w)

    com_is_br(w) := valid && uop.is_br
    com_is_jal(w) := valid && uop.is_jal
    com_is_jalr(w)  := valid && uop.is_jalr
    // 这里遵从了香山的 PreDecode.scala 中检测 call 和 ret 的逻辑, 它和
    // RISC-V 基本指令集手册的 2.5.1. Unconditional Jumps 一节定义的
    // call/ret 稍微有些出入。RISC-V 手册中允许 rs1 和 rd 都是 x1 或
    // x5 来表达又是 call 又是 ret 的情况，但这种 case 处理起来会有些麻烦
    com_is_call(w) := valid && (uop.is_jalr || uop.is_jal) && ((uop.ldst === 1.U) || (uop.ldst === 5.U)) // c.jal 在 RV64 中会被译码为 c.addiw
    com_is_ret(w)   := valid && uop.is_jalr && ((uop.lrs1 === 1.U) || (uop.lrs1 === 5.U)) && !com_is_call(w)

    // fsrc 表示 next pc 从哪来的，如果 next pc 来自后端，则说明分支预测错误
    com_misp_br(w)    := com_is_br(w)   && uop.debug_fsrc === BSRC_C
    com_misp_jalr(w)  := com_is_jalr(w) && uop.debug_fsrc === BSRC_C 
    com_misp_ret(w)   := com_is_ret(w)  && uop.debug_fsrc === BSRC_C 
    // jal 在至多在 f3 阶段就能确定跳转目标
  }

  when (startCounter && useEventCounter.B) {
    event_counters.io.event_signals(0) :=   1.U  //cycles
    event_counters.io.event_signals(1) :=  RegNext(PopCount(rob.io.commit.arch_valids.asUInt)) // commit inst
    // TODO: 因为 BOOM 前端的 replay 机制，计数器 cache valid access 与 tlb valid access 的值
    // 会比实际的多，但 hit number 是准的，实际的 access time = hit number + perf miss？
    // 但这还是有点微妙，例如 icache miss 但 itlb hit 的情况，replay 会导致 itlb 反复 hit
    event_counters.io.event_signals(2) :=  Mux(io.ifu.icache_valid_access, 1.U, 0.U) //i-cache valid access number
    event_counters.io.event_signals(3) :=  Mux(io.ifu.icache_hit, 1.U, 0.U)  //icache hit number
    event_counters.io.event_signals(4) :=  Mux(io.ifu.perf.acquire, 1.U, 0.U) //i-cache send req to next level cache
    event_counters.io.event_signals(5) :=  Mux(io.ifu.itlb_valid_access, 1.U, 0.U) //itlb valid access number
    event_counters.io.event_signals(6) :=  Mux(io.ifu.itlb_hit, 1.U, 0.U) //itlb hit number
    event_counters.io.event_signals(7) :=  Mux(io.ifu.perf.tlbMiss, 1.U, 0.U) //i-tlb start ptw

    event_counters.io.event_signals(8) :=  PopCount(com_is_br.asUInt)       //committed br number
    event_counters.io.event_signals(9) :=  PopCount(com_is_jalr.asUInt)   // committed jalr number
    event_counters.io.event_signals(10) :=  PopCount(com_is_ret.asUInt)     //committed jalr-ret number
    event_counters.io.event_signals(11) :=  PopCount(com_is_jal.asUInt)  //committed jal number
    event_counters.io.event_signals(12) :=  PopCount(com_is_call.asUInt)  //committed call number
    event_counters.io.event_signals(13) :=  PopCount(com_misp_br.asUInt)       //committed misp br number
    event_counters.io.event_signals(14) :=  PopCount(com_misp_jalr.asUInt)   //committed misp jalr number
    event_counters.io.event_signals(15) :=  PopCount(com_misp_ret.asUInt)     //committed misp jalr-ret number

    event_counters.io.event_signals(16) :=  com_ft_bsrc1 // committed fetch packet from f1 prediction/no bubble
    event_counters.io.event_signals(17) :=  com_ft_bsrc2 // committed fetch packet from f2 prediction/one bubble 
    event_counters.io.event_signals(18) :=  com_ft_bsrc3 // committed fetch packet from f3 prediction/two bubbles
    event_counters.io.event_signals(19) :=  com_ft_bsrcc // committed fetch packet from backend correction/many bubbles

    // 39: ROB flushes caused by MINI_EXCEPTION_MEM_ORDERING
    event_counters.io.event_signals(39) := Mux(rob_mem_ordering_flush, 1.U, 0.U)

    // Decode valid pattern profiling
    // 40: all dec_valids are false
    // 41: all dec_valids are true
    // 42: mixed dec_valids (neither all false nor all true)
    event_counters.io.event_signals(40) := Mux(dec_all_false, 1.U, 0.U)
    event_counters.io.event_signals(41) := Mux(dec_all_true, 1.U, 0.U)
    event_counters.io.event_signals(42) := Mux(!dec_all_false && !dec_all_true, 1.U, 0.U)

    // Fetch buffer enqueue profiling
    // 20: no enqueue this cycle
    // 21: enqueue with 0 instructions
    // 22: enqueue with 1 instruction
    // 23: enqueue with 2 instructions
    // 24: enqueue with 3 instructions
    // 25: enqueue with 4 instructions
    event_counters.io.event_signals(20) := Mux(!fb_enq_fire, 1.U, 0.U)
    event_counters.io.event_signals(21) := Mux(fb_enq_fire && fb_enq_cnt === 0.U, 1.U, 0.U)
    event_counters.io.event_signals(22) := Mux(fb_enq_fire && fb_enq_cnt === 1.U, 1.U, 0.U)
    event_counters.io.event_signals(23) := Mux(fb_enq_fire && fb_enq_cnt === 2.U, 1.U, 0.U)
    event_counters.io.event_signals(24) := Mux(fb_enq_fire && fb_enq_cnt === 3.U, 1.U, 0.U)
    event_counters.io.event_signals(25) := Mux(fb_enq_fire && fb_enq_cnt === 4.U, 1.U, 0.U)

    // Frontend s2 replay statistics
    // 26: all s2 replays
    // 27: s2 replays due to itlb miss
    // 28: s2 replays due to icache miss with itlb hit
    event_counters.io.event_signals(26) := Mux(s2_replay_total, 1.U, 0.U)
    event_counters.io.event_signals(27) := Mux(s2_replay_itlb_miss, 1.U, 0.U)
    event_counters.io.event_signals(28) := Mux(s2_replay_ic_miss, 1.U, 0.U)

    // 29: dcache miss (LSU acquire)
    event_counters.io.event_signals(29) := Mux(io.lsu.perf.acquire, 1.U, 0.U)

    // 30-36: BPD-ahead-of-IFU FTQ pointer distance buckets
    // 30: distance 0-1
    // 31: distance 2-3
    // 32: distance 4-5
    // 33: distance 6-8
    // 34: distance 9-12
    // 35: distance 13-16
    // 36: distance >16
    event_counters.io.event_signals(30) := Mux(bpd_ifu_dist_bucket === 0.U, 1.U, 0.U)
    event_counters.io.event_signals(31) := Mux(bpd_ifu_dist_bucket === 1.U, 1.U, 0.U)
    event_counters.io.event_signals(32) := Mux(bpd_ifu_dist_bucket === 2.U, 1.U, 0.U)
    event_counters.io.event_signals(33) := Mux(bpd_ifu_dist_bucket === 3.U, 1.U, 0.U)
    event_counters.io.event_signals(34) := Mux(bpd_ifu_dist_bucket === 4.U, 1.U, 0.U)
    event_counters.io.event_signals(35) := Mux(bpd_ifu_dist_bucket === 5.U, 1.U, 0.U)
    event_counters.io.event_signals(36) := Mux(bpd_ifu_dist_bucket === 6.U, 1.U, 0.U)

    // 37-38: frontend s0 stall statistics
    // 37: s0_ifu_real_valid is false
    // 38: s0_valid && ifu_to_ftq_not_ready
    event_counters.io.event_signals(37) := Mux(s0_ifu_real_not_valid, 1.U, 0.U)
    event_counters.io.event_signals(38) := Mux(s0_ifu_ftq_backpress, 1.U, 0.U)

    // 43-49: frontend bubble statistics by clear source
    // 43: bubbles from F2 clear (low 4 bits)
    // 44: bubbles from F3 clear (low 4 bits)
    // 45: bubbles from predecode clear (low 4 bits)
    // 46: backend ROB flush bubbles (low 4 bits)
    // 47: backend ROB flush bubbles (high 2 bits)
    // 48: backend mispredict flush bubbles (low 4 bits)
    // 49: backend mispredict flush bubbles (high 2 bits)
    event_counters.io.event_signals(43) := f2_clear_bubble
    event_counters.io.event_signals(44) := f3_clear_bubble
    event_counters.io.event_signals(45) := predecode_clear_bubble

    val rob_bubble_low  = rob_flush_bubble(3,0)
    val rob_bubble_high = rob_flush_bubble(5,4)
    event_counters.io.event_signals(46) := rob_bubble_low
    event_counters.io.event_signals(47) := rob_bubble_high

    val mispred_bubble_low  = mispred_flush_bubble(3,0)
    val mispred_bubble_high = mispred_flush_bubble(5,4)
    event_counters.io.event_signals(48) := mispred_bubble_low
    event_counters.io.event_signals(49) := mispred_bubble_high

    // 50-56: Prefetch-ahead-of-IFU distance buckets at prefetch refill completion
    // Only counted when a prefetch MSHR completes writeback and is not cancelled by fencei
    // 50: distance 0-1
    // 51: distance 2-3
    // 52: distance 4-5
    // 53: distance 6-8
    // 54: distance 9-12
    // 55: distance 13-16
    // 56: distance >16
    event_counters.io.event_signals(50) := Mux(pf_refill_dist_bucket.valid && pf_refill_dist_bucket.bits === 0.U, 1.U, 0.U)
    event_counters.io.event_signals(51) := Mux(pf_refill_dist_bucket.valid && pf_refill_dist_bucket.bits === 1.U, 1.U, 0.U)
    event_counters.io.event_signals(52) := Mux(pf_refill_dist_bucket.valid && pf_refill_dist_bucket.bits === 2.U, 1.U, 0.U)
    event_counters.io.event_signals(53) := Mux(pf_refill_dist_bucket.valid && pf_refill_dist_bucket.bits === 3.U, 1.U, 0.U)
    event_counters.io.event_signals(54) := Mux(pf_refill_dist_bucket.valid && pf_refill_dist_bucket.bits === 4.U, 1.U, 0.U)
    event_counters.io.event_signals(55) := Mux(pf_refill_dist_bucket.valid && pf_refill_dist_bucket.bits === 5.U, 1.U, 0.U)
    event_counters.io.event_signals(56) := Mux(pf_refill_dist_bucket.valid && pf_refill_dist_bucket.bits === 6.U, 1.U, 0.U)

    event_counters.io.event_signals(57) := Mux(io.lsu.perf.tlbMiss, 1.U, 0.U) // dtlb miss

    // 58: successful prefetch count (prefetch-filled line first accessed by IFU)
    event_counters.io.event_signals(58) := Mux(io.ifu.pf_hit_success, 1.U, 0.U)
  }

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Fetch Stage/Frontend ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------
  io.ifu.redirect_val         := false.B
  io.ifu.redirect_flush       := false.B
  io.ifu.rob_flush            := false.B
  io.ifu.rob_flush_pc_lob     := 0.U

  // Breakpoint info
  io.ifu.status  := csr.io.status
  io.ifu.bp      := csr.io.bp
  io.ifu.mcontext := csr.io.mcontext
  io.ifu.scontext := csr.io.scontext

  io.ifu.flush_icache := (0 until coreWidth).map { i =>
    (rob.io.commit.arch_valids(i) && rob.io.commit.uops(i).is_fencei) ||
    (RegNext(dec_valids(i) && dec_uops(i).is_jalr && csr.io.status.debug))
  }.reduce(_||_)

  // TODO FIX THIS HACK
  // The below code works because of two quirks with the flush mechanism
  //  1 ) All flush_on_commit instructions are also is_unique,
  //      In the future, this constraint will be relaxed.
  //  2 ) We send out flush signals one cycle after the commit signal. We need to
  //      mux between one/two cycle delay for the following cases:
  //       ERETs are reported to the CSR two cycles before we send the flush
  //       Exceptions are reported to the CSR on the cycle we send the flush
  // This discrepency should be resolved elsewhere.
  when (RegNext(rob.io.flush.valid)) {
    io.ifu.rob_flush      := true.B
    io.ifu.rob_flush_pc_lob := RegNext(rob.io.flush.bits.pc_lob)
    io.ifu.redirect_val   := true.B
    io.ifu.redirect_flush := true.B
    val flush_typ = RegNext(rob.io.flush.bits.flush_typ)
    // Clear the global history when we flush the ROB (exceptions, AMOs, unique instructions, etc.)
    val new_ghist = WireInit((0.U).asTypeOf(new GlobalHistory))
    new_ghist.current_saw_branch_not_taken := true.B
    new_ghist.ras_idx := io.ifu.get_pc(0).entry.ras_idx
    io.ifu.redirect_ghist := new_ghist
    when (FlushTypes.useCsrEvec(flush_typ)) {
      //Enable_Sample_Support
      when (overflow_event && RegNext(rob.io.com_xcpt.valid && rob.io.com_xcpt.bits.cause === Cause_OverFlow)) {
        when (exitFuncAddr =/= 0.U ) {  //redirected to the setting target addr
          io.ifu.redirect_pc  := exitFuncAddr
          procTag := 0.U
        }
        .otherwise {        // trigger a ecall, which need to be captured by ptrace
          io.ifu.redirect_pc  := csr.io.evec
          sampleHappen := 1.U   // set the register for ptrace detect the sample ecall
        }
        maxEventNum := 0.U
        nowEventNum := 0.U
        pfc_enable  := 0.U     // disable the performance counter
      }
      .otherwise {
        io.ifu.redirect_pc  := Mux(flush_typ === FlushTypes.eret,
                                  RegNext(RegNext(csr.io.evec)),
                                  csr.io.evec)
      }
    } .otherwise {
      val flush_pc = (AlignPCToBoundary(io.ifu.get_pc(0).pc, icBlockBytes)
                      + RegNext(rob.io.flush.bits.pc_lob)
                      - Mux(RegNext(rob.io.flush.bits.edge_inst), 2.U, 0.U))
      val flush_pc_next = flush_pc + Mux(RegNext(rob.io.flush.bits.is_rvc), 2.U, 4.U)
      //Enable_Sample_Support
      when (RegNext(rob.io.flush.bits.isURet)) {
        switch (RegNext(rob.io.flush.bits.uret_target)) {
          is (0.U)  { io.ifu.redirect_pc := tempReg1 }
          is (1.U)  { io.ifu.redirect_pc := tempReg2 }
          is (2.U)  { io.ifu.redirect_pc := tempReg3 }
          is (3.U)  { io.ifu.redirect_pc := tempReg4 }
        }
      }
      .otherwise {
        io.ifu.redirect_pc := Mux(FlushTypes.useSamePC(flush_typ),
                                  flush_pc, flush_pc_next)
      // Restore ghist for refetch/next flush types instead of resetting to zero
      // 没有处理 flush on commit 之后接下一个 fetch packet 这种情况（这时候 history 可能需要移一个 0 才对）
      if (enableFlushGHistRestore) {
        val ftq_entry_flush = io.ifu.get_pc(0).entry
        val flush_pc_lob_r = RegNext(rob.io.flush.bits.pc_lob)
        val flush_idx = (flush_pc_lob_r ^
          Mux(ftq_entry_flush.start_bank === 1.U, 1.U << log2Ceil(bankBytes), 0.U))(log2Ceil(fetchWidth), 1)
        val flush_idx_oh = UIntToOH(flush_idx)
        val flush_mask = MaskLower(flush_idx_oh)
        val has_br_before_flush = (flush_mask & ftq_entry_flush.br_mask) =/= 0.U
        new_ghist.old_history := io.ifu.get_pc(1).ghist.old_history
        new_ghist.current_saw_branch_not_taken := has_br_before_flush
      }
      }
    }
    io.ifu.redirect_ftq_idx := RegNext(rob.io.flush.bits.ftq_idx)
  } .elsewhen (brupdate.b2.mispredict && !RegNext(rob.io.flush.valid)) {
    val block_pc = AlignPCToBoundary(io.ifu.get_pc(1).pc, icBlockBytes)
    val uop_maybe_pc = block_pc | brupdate.b2.uop.pc_lob
    val npc = uop_maybe_pc + Mux(brupdate.b2.uop.is_rvc || brupdate.b2.uop.edge_inst, 2.U, 4.U)
    val jal_br_target = Wire(UInt(vaddrBitsExtended.W))
    jal_br_target := (uop_maybe_pc.asSInt + brupdate.b2.target_offset +
      (Fill(vaddrBitsExtended-1, brupdate.b2.uop.edge_inst) << 1).asSInt).asUInt
    val bj_addr = Mux(brupdate.b2.cfi_type === CFI_JALR, brupdate.b2.jalr_target, jal_br_target)
    val mispredict_target = Mux(brupdate.b2.pc_sel === PC_PLUS4, npc, bj_addr)
    io.ifu.redirect_val     := true.B
    io.ifu.redirect_pc      := mispredict_target
    io.ifu.redirect_flush   := true.B
    io.ifu.redirect_ftq_idx := brupdate.b2.uop.ftq_idx
    val use_same_ghist = (brupdate.b2.cfi_type === CFI_BR &&
                          !brupdate.b2.taken &&
                          bankAlign(block_pc) === bankAlign(npc))
    val ftq_entry = io.ifu.get_pc(1).entry
    val cfi_idx = (brupdate.b2.uop.pc_lob ^
      Mux(ftq_entry.start_bank === 1.U, 1.U << log2Ceil(bankBytes), 0.U))(log2Ceil(fetchWidth), 1)
    val ftq_ghist = io.ifu.get_pc(1).ghist
    val (next_ghist, _) = ftq_ghist.update(
      ftq_entry.br_mask.asUInt,
      brupdate.b2.taken,
      brupdate.b2.cfi_type === CFI_BR,
      cfi_idx,
      true.B,
      io.ifu.get_pc(1).pc,
      ftq_entry.cfi_is_call && ftq_entry.cfi_idx.bits === cfi_idx,
      ftq_entry.cfi_is_ret  && ftq_entry.cfi_idx.bits === cfi_idx)


    io.ifu.redirect_ghist   := Mux(
      use_same_ghist,
      ftq_ghist,
      next_ghist)
    io.ifu.redirect_ghist.current_saw_branch_not_taken := use_same_ghist
    // 在 decoupled frontend 里，ghist 是从 f3 preds 写入到 ftq，它的 ras idx
    // 不一定准确
    val redirect_cfi_is_call = ftq_entry.cfi_is_call && ftq_entry.cfi_idx.bits === cfi_idx
    when (redirect_cfi_is_call) {
      io.ifu.redirect_ghist.ras_idx := WrapInc(ftq_entry.ras_idx, nRasEntries)
    } .elsewhen( ftq_entry.cfi_is_ret && ftq_entry.cfi_idx.bits === cfi_idx) {
      io.ifu.redirect_ghist.ras_idx := WrapDec(ftq_entry.ras_idx, nRasEntries)
    } .otherwise {
      io.ifu.redirect_ghist.ras_idx := ftq_entry.ras_idx
    }
    io.ifu.redirect_is_call   := redirect_cfi_is_call
    io.ifu.redirect_call_addr := npc
  } .elsewhen (rob.io.flush_frontend || brupdate.b1.mispredict_mask =/= 0.U) {
    io.ifu.redirect_flush   := true.B
  }

  // Tell the FTQ it can deallocate entries by passing youngest ftq_idx.
  val youngest_com_idx = (coreWidth-1).U - PriorityEncoder(rob.io.commit.valids.reverse)
  io.ifu.commit.valid := rob.io.commit.valids.reduce(_|_) || rob.io.com_xcpt.valid
  io.ifu.commit.bits  := Mux(rob.io.com_xcpt.valid,
                             rob.io.com_xcpt.bits.ftq_idx,
                             rob.io.commit.uops(youngest_com_idx).ftq_idx)

  assert(!(rob.io.commit.valids.reduce(_|_) && rob.io.com_xcpt.valid),
    "ROB can't commit and except in same cycle!")

  for (i <- 0 until memWidth) {
    when (RegNext(io.lsu.exe(i).req.bits.sfence.valid)) {
      io.ifu.sfence := RegNext(io.lsu.exe(i).req.bits.sfence)
    }
  }

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Branch Prediction ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Decode Stage ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  // track mask of finished instructions in the bundle
  // use this to mask out insts coming from FetchBuffer that have been finished
  // for example, back pressure may cause us to only issue some instructions from FetchBuffer
  // but on the next cycle, we only want to retry a subset
  val dec_finished_mask = RegInit(0.U(coreWidth.W))

  //-------------------------------------------------------------
  // Pull out instructions and send to the Decoders

  io.ifu.fetchpacket.ready := dec_ready
  val dec_fbundle = io.ifu.fetchpacket.bits

  //-------------------------------------------------------------
  // Decoders

  for (w <- 0 until coreWidth) {
    dec_valids(w)                      := io.ifu.fetchpacket.valid && dec_fbundle.uops(w).valid &&
                                          !dec_finished_mask(w)
    decode_units(w).io.enq.uop         := dec_fbundle.uops(w).bits
    decode_units(w).io.status          := csr.io.status
    decode_units(w).io.csr_decode      <> csr.io.decode(w)
    decode_units(w).io.interrupt       := csr.io.interrupt
    decode_units(w).io.interrupt_cause := csr.io.interrupt_cause

    dec_uops(w) := decode_units(w).io.deq.uop

    ////Enable_Sample_Support: when overflow maxinsts will make all decoded uop become illegal inst
    when (overflow_event) {
      dec_uops(w).exception := true.B
      dec_uops(w).exc_cause := Cause_OverFlow
    }
  }

  //-------------------------------------------------------------
  // FTQ GetPC Port Arbitration

  val jmp_pc_req  = Wire(Decoupled(UInt(log2Ceil(ftqSz).W)))
  val xcpt_pc_req = Wire(Decoupled(UInt(log2Ceil(ftqSz).W)))
  val flush_pc_req = Wire(Decoupled(UInt(log2Ceil(ftqSz).W)))

  val ftq_arb = Module(new Arbiter(UInt(log2Ceil(ftqSz).W), 3))

  // Order by the oldest. Flushes come from the oldest instructions in pipe
  // Decoding exceptions come from youngest
  ftq_arb.io.in(0) <> flush_pc_req
  ftq_arb.io.in(1) <> jmp_pc_req
  ftq_arb.io.in(2) <> xcpt_pc_req

  // Hookup FTQ
  io.ifu.get_pc(0).ftq_idx := ftq_arb.io.out.bits
  ftq_arb.io.out.ready  := true.B

  // Branch Unit Requests (for JALs) (Should delay issue of JALs if this not ready)
  jmp_pc_req.valid := RegNext(iss_valids(jmp_unit_idx) && iss_uops(jmp_unit_idx).fu_code === FU_JMP)
  jmp_pc_req.bits  := RegNext(iss_uops(jmp_unit_idx).ftq_idx)

  jmp_unit.io.get_ftq_pc := DontCare
  jmp_unit.io.get_ftq_pc.pc               := io.ifu.get_pc(0).pc
  jmp_unit.io.get_ftq_pc.entry            := io.ifu.get_pc(0).entry
  jmp_unit.io.get_ftq_pc.next_val         := io.ifu.get_pc(0).next_val
  jmp_unit.io.get_ftq_pc.next_pc          := io.ifu.get_pc(0).next_pc


  // Frontend Exception Requests
  val xcpt_idx = PriorityEncoder(dec_xcpts)
  xcpt_pc_req.valid    := dec_xcpts.reduce(_||_)
  xcpt_pc_req.bits     := dec_uops(xcpt_idx).ftq_idx
  //rob.io.xcpt_fetch_pc := RegEnable(io.ifu.get_pc.fetch_pc, dis_ready)
  rob.io.xcpt_fetch_pc := io.ifu.get_pc(0).pc

  flush_pc_req.valid   := rob.io.flush.valid
  flush_pc_req.bits    := rob.io.flush.bits.ftq_idx

  // Mispredict requests (to get the correct target)
  // When ROB flush is valid and enableFlushGHistRestore, steal get_pc(1) to read ghist for the flushing instruction
  if (enableFlushGHistRestore) {
    io.ifu.get_pc(1).ftq_idx := Mux(rob.io.flush.valid, rob.io.flush.bits.ftq_idx, oldest_mispredict_ftq_idx)
  } else {
    io.ifu.get_pc(1).ftq_idx := oldest_mispredict_ftq_idx
  }
  io.ifu.mispredict_val    := mispredict_val


  //-------------------------------------------------------------
  // Decode/Rename1 pipeline logic

  dec_xcpts := dec_uops zip dec_valids map {case (u,v) => u.exception && v}
  val dec_xcpt_stall = dec_xcpts.reduce(_||_) && !xcpt_pc_req.ready
  // stall fetch/dcode because we ran out of branch tags
  val branch_mask_full = Wire(Vec(coreWidth, Bool()))

  val dec_hazards = (0 until coreWidth).map(w =>
                      dec_valids(w) &&
                      (  !dis_ready
                      || rob.io.commit.rollback
                      || dec_xcpt_stall
                      || branch_mask_full(w)
                      || brupdate.b1.mispredict_mask =/= 0.U
                      || brupdate.b2.mispredict
                      || io.ifu.redirect_flush))

  val dec_stalls = dec_hazards.scanLeft(false.B) ((s,h) => s || h).takeRight(coreWidth)
  dec_fire := (0 until coreWidth).map(w => dec_valids(w) && !dec_stalls(w))

  // all decoders are empty and ready for new instructions
  dec_ready := dec_fire.last

  when (dec_ready || io.ifu.redirect_flush) {
    dec_finished_mask := 0.U
  } .otherwise {
    dec_finished_mask := dec_fire.asUInt | dec_finished_mask
  }

  //-------------------------------------------------------------
  // Branch Mask Logic

  dec_brmask_logic.io.brupdate := brupdate
  dec_brmask_logic.io.flush_pipeline := RegNext(rob.io.flush.valid)

  for (w <- 0 until coreWidth) {
    dec_brmask_logic.io.is_branch(w) := !dec_finished_mask(w) && dec_uops(w).allocate_brtag
    dec_brmask_logic.io.will_fire(w) :=  dec_fire(w) &&
                                         dec_uops(w).allocate_brtag // ren, dis can back pressure us
    dec_uops(w).br_tag  := dec_brmask_logic.io.br_tag(w)
    dec_uops(w).br_mask := dec_brmask_logic.io.br_mask(w)
  }

  branch_mask_full := dec_brmask_logic.io.is_full

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Register Rename Stage ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  // Inputs
  for (rename <- rename_stages) {
    rename.io.kill := io.ifu.redirect_flush
    rename.io.brupdate := brupdate

    rename.io.debug_rob_empty := rob.io.empty

    rename.io.dec_fire := dec_fire
    rename.io.dec_uops := dec_uops

    rename.io.dis_fire := dis_fire
    rename.io.dis_ready := dis_ready

    rename.io.com_valids := rob.io.commit.valids
    rename.io.com_uops := rob.io.commit.uops
    rename.io.rbk_valids := rob.io.commit.rbk_valids
    rename.io.rollback := rob.io.commit.rollback
  }


  // Outputs
  dis_uops := rename_stage.io.ren2_uops
  dis_valids := rename_stage.io.ren2_mask
  ren_stalls := rename_stage.io.ren_stalls


  /**
   * TODO This is a bit nasty, but it's currently necessary to
   * split the INT/FP rename pipelines into separate instantiations.
   * Won't have to do this anymore with a properly decoupled FP pipeline.
   */
  for (w <- 0 until coreWidth) {
    val i_uop   = rename_stage.io.ren2_uops(w)
    val f_uop   = if (usingFPU) fp_rename_stage.io.ren2_uops(w) else NullMicroOp
    val p_uop   = if (enableSFBOpt) pred_rename_stage.io.ren2_uops(w) else NullMicroOp
    val f_stall = if (usingFPU) fp_rename_stage.io.ren_stalls(w) else false.B
    val p_stall = if (enableSFBOpt) pred_rename_stage.io.ren_stalls(w) else false.B

    // lrs1 can "pass through" to prs1. Used solely to index the csr file.
    dis_uops(w).prs1 := Mux(dis_uops(w).lrs1_rtype === RT_FLT, f_uop.prs1,
                        Mux(dis_uops(w).lrs1_rtype === RT_FIX, i_uop.prs1, dis_uops(w).lrs1))
    dis_uops(w).prs2 := Mux(dis_uops(w).lrs2_rtype === RT_FLT, f_uop.prs2, i_uop.prs2)
    dis_uops(w).prs3 := f_uop.prs3
    dis_uops(w).ppred := p_uop.ppred
    dis_uops(w).pdst := Mux(dis_uops(w).dst_rtype  === RT_FLT, f_uop.pdst,
                        Mux(dis_uops(w).dst_rtype  === RT_FIX, i_uop.pdst,
                                                               p_uop.pdst))
    dis_uops(w).stale_pdst := Mux(dis_uops(w).dst_rtype === RT_FLT, f_uop.stale_pdst, i_uop.stale_pdst)

    dis_uops(w).prs1_busy := i_uop.prs1_busy && (dis_uops(w).lrs1_rtype === RT_FIX) ||
                             f_uop.prs1_busy && (dis_uops(w).lrs1_rtype === RT_FLT)
    dis_uops(w).prs2_busy := i_uop.prs2_busy && (dis_uops(w).lrs2_rtype === RT_FIX) ||
                             f_uop.prs2_busy && (dis_uops(w).lrs2_rtype === RT_FLT)
    dis_uops(w).prs3_busy := f_uop.prs3_busy && dis_uops(w).frs3_en
    dis_uops(w).ppred_busy := p_uop.ppred_busy && dis_uops(w).is_sfb_shadow

    ren_stalls(w) := rename_stage.io.ren_stalls(w) || f_stall || p_stall
  }

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Dispatch Stage ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  //-------------------------------------------------------------
  // Rename2/Dispatch pipeline logic

  val dis_prior_slot_valid = dis_valids.scanLeft(false.B) ((s,v) => s || v)
  val dis_prior_slot_unique = (dis_uops zip dis_valids).scanLeft(false.B) {case (s,(u,v)) => s || v && u.is_unique}
  val wait_for_empty_pipeline = (0 until coreWidth).map(w => (dis_uops(w).is_unique || custom_csrs.disableOOO) &&
                                  (!rob.io.empty || !io.lsu.fencei_rdy || dis_prior_slot_valid(w)))
  val rocc_shim_busy = if (usingRoCC) !exe_units.rocc_unit.io.rocc.rxq_empty else false.B
  val wait_for_rocc = (0 until coreWidth).map(w =>
                        (dis_uops(w).is_fence || dis_uops(w).is_fencei) && (io.rocc.busy || rocc_shim_busy))
  val rxq_full = if (usingRoCC) exe_units.rocc_unit.io.rocc.rxq_full else false.B
  val block_rocc = (dis_uops zip dis_valids).map{case (u,v) => v && u.uopc === uopROCC}.scanLeft(rxq_full)(_||_)
  val dis_rocc_alloc_stall = (dis_uops.map(_.uopc === uopROCC) zip block_rocc) map {case (p,r) =>
                               if (usingRoCC) p && r else false.B}

  val dis_hazards = (0 until coreWidth).map(w =>
                      dis_valids(w) &&
                      (  !rob.io.ready
                      || ren_stalls(w)
                      || io.lsu.ldq_full(w) && dis_uops(w).uses_ldq
                      || io.lsu.stq_full(w) && dis_uops(w).uses_stq
                      || !dispatcher.io.ren_uops(w).ready
                      || wait_for_empty_pipeline(w)
                      || wait_for_rocc(w)
                      || dis_prior_slot_unique(w)
                      || dis_rocc_alloc_stall(w)
                      || brupdate.b1.mispredict_mask =/= 0.U
                      || brupdate.b2.mispredict
                      || io.ifu.redirect_flush))


  io.lsu.fence_dmem := (dis_valids zip wait_for_empty_pipeline).map {case (v,w) => v && w} .reduce(_||_)

  val dis_stalls = dis_hazards.scanLeft(false.B) ((s,h) => s || h).takeRight(coreWidth)
  dis_fire := dis_valids zip dis_stalls map {case (v,s) => v && !s}
  dis_ready := !dis_stalls.last

  //-------------------------------------------------------------
  // LDQ/STQ Allocation Logic

  for (w <- 0 until coreWidth) {
    // Dispatching instructions request load/store queue entries when they can proceed.
    dis_uops(w).ldq_idx := io.lsu.dis_ldq_idx(w)
    dis_uops(w).stq_idx := io.lsu.dis_stq_idx(w)
  }

  //-------------------------------------------------------------
  // Rob Allocation Logic

  rob.io.enq_valids := dis_fire
  rob.io.enq_uops   := dis_uops
  rob.io.enq_partial_stall := dis_stalls.last // TODO come up with better ROB compacting scheme.
  rob.io.debug_tsc := debug_tsc_reg
  rob.io.csr_stall := csr.io.csr_stall

  // Minor hack: ecall and breaks need to increment the FTQ deq ptr earlier than commit, since
  // they write their PC into the CSR the cycle before they commit.
  // Since these are also unique, increment the FTQ ptr when they are dispatched
  when (RegNext(dis_fire.reduce(_||_) && dis_uops(PriorityEncoder(dis_fire)).is_sys_pc2epc)) {
    io.ifu.commit.valid := true.B
    io.ifu.commit.bits  := RegNext(dis_uops(PriorityEncoder(dis_valids)).ftq_idx)
  }

  for (w <- 0 until coreWidth) {
    // note: this assumes uops haven't been shifted - there's a 1:1 match between PC's LSBs and "w" here
    // (thus the LSB of the rob_idx gives part of the PC)
    if (coreWidth == 1) {
      dis_uops(w).rob_idx := rob.io.rob_tail_idx
    } else {
      dis_uops(w).rob_idx := Cat(rob.io.rob_tail_idx >> log2Ceil(coreWidth).U,
                               w.U(log2Ceil(coreWidth).W))
    }
  }

  //-------------------------------------------------------------
  // RoCC allocation logic
  if (usingRoCC) {
    for (w <- 0 until coreWidth) {
      // We guarantee only decoding 1 RoCC instruction per cycle
      dis_uops(w).rxq_idx := exe_units.rocc_unit.io.rocc.rxq_idx(w)
    }
  }

  //-------------------------------------------------------------
  // Dispatch to issue queues

  // Get uops from rename2
  for (w <- 0 until coreWidth) {
    dispatcher.io.ren_uops(w).valid := dis_fire(w)
    dispatcher.io.ren_uops(w).bits  := dis_uops(w)
  }

  var iu_idx = 0
  // Send dispatched uops to correct issue queues
  // Backpressure through dispatcher if necessary
  for (i <- 0 until issueParams.size) {
    if (issueParams(i).iqType == IQT_FP.litValue) {
       fp_pipeline.io.dis_uops <> dispatcher.io.dis_uops(i)
    } else {
       issue_units(iu_idx).io.dis_uops <> dispatcher.io.dis_uops(i)
       iu_idx += 1
    }
  }

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Issue Stage ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  require (issue_units.map(_.issueWidth).sum == exe_units.length)

  var iss_wu_idx = 1
  var ren_wu_idx = 1
  // The 0th wakeup port goes to the ll_wbarb
  int_iss_wakeups(0).valid := ll_wbarb.io.out.fire && ll_wbarb.io.out.bits.uop.dst_rtype === RT_FIX
  int_iss_wakeups(0).bits  := ll_wbarb.io.out.bits

  int_ren_wakeups(0).valid := ll_wbarb.io.out.fire && ll_wbarb.io.out.bits.uop.dst_rtype === RT_FIX
  int_ren_wakeups(0).bits  := ll_wbarb.io.out.bits

  for (i <- 1 until memWidth) {
    int_iss_wakeups(i).valid := mem_resps(i).valid && mem_resps(i).bits.uop.dst_rtype === RT_FIX
    int_iss_wakeups(i).bits  := mem_resps(i).bits

    int_ren_wakeups(i).valid := mem_resps(i).valid && mem_resps(i).bits.uop.dst_rtype === RT_FIX
    int_ren_wakeups(i).bits  := mem_resps(i).bits
    iss_wu_idx += 1
    ren_wu_idx += 1
  }

  // loop through each issue-port (exe_units are statically connected to an issue-port)
  for (i <- 0 until exe_units.length) {
    if (exe_units(i).writesIrf) {
      val fast_wakeup = Wire(Valid(new ExeUnitResp(xLen)))
      val slow_wakeup = Wire(Valid(new ExeUnitResp(xLen)))
      fast_wakeup := DontCare
      slow_wakeup := DontCare

      val resp = exe_units(i).io.iresp
      assert(!(resp.valid && resp.bits.uop.rf_wen && resp.bits.uop.dst_rtype =/= RT_FIX))

      // Fast Wakeup (uses just-issued uops that have known latencies)
      fast_wakeup.bits.uop := iss_uops(i)
      fast_wakeup.valid    := iss_valids(i) &&
                              iss_uops(i).bypassable &&
                              iss_uops(i).dst_rtype === RT_FIX &&
                              iss_uops(i).ldst_val &&
                              !(io.lsu.ld_miss && (iss_uops(i).iw_p1_poisoned || iss_uops(i).iw_p2_poisoned))

      // Slow Wakeup (uses write-port to register file)
      slow_wakeup.bits.uop := resp.bits.uop
      slow_wakeup.valid    := resp.valid &&
                                resp.bits.uop.rf_wen &&
                                !resp.bits.uop.bypassable &&
                                resp.bits.uop.dst_rtype === RT_FIX

      if (exe_units(i).bypassable) {
        int_iss_wakeups(iss_wu_idx) := fast_wakeup
        iss_wu_idx += 1
      }
      if (!exe_units(i).alwaysBypassable) {
        int_iss_wakeups(iss_wu_idx) := slow_wakeup
        iss_wu_idx += 1
      }

      if (exe_units(i).bypassable) {
        int_ren_wakeups(ren_wu_idx) := fast_wakeup
        ren_wu_idx += 1
      }
      if (!exe_units(i).alwaysBypassable) {
        int_ren_wakeups(ren_wu_idx) := slow_wakeup
        ren_wu_idx += 1
      }
    }
  }
  require (iss_wu_idx == numIntIssueWakeupPorts)
  require (ren_wu_idx == numIntRenameWakeupPorts)
  require (iss_wu_idx == ren_wu_idx)

  // jmp unit performs fast wakeup of the predicate bits
  require (jmp_unit.bypassable)
  pred_wakeup.valid := (iss_valids(jmp_unit_idx) &&
                        iss_uops(jmp_unit_idx).is_sfb_br &&
                        !(io.lsu.ld_miss && (iss_uops(jmp_unit_idx).iw_p1_poisoned || iss_uops(jmp_unit_idx).iw_p2_poisoned))
  )
  pred_wakeup.bits.uop := iss_uops(jmp_unit_idx)
  pred_wakeup.bits.fflags := DontCare
  pred_wakeup.bits.data := DontCare
  pred_wakeup.bits.predicated := DontCare

  // Perform load-hit speculative wakeup through a special port (performs a poison wake-up).
  issue_units map { iu =>
     iu.io.spec_ld_wakeup := io.lsu.spec_ld_wakeup
  }


  // Connect the predicate wakeup port
  issue_units map { iu =>
    iu.io.pred_wakeup_port.valid := false.B
    iu.io.pred_wakeup_port.bits := DontCare
  }
  if (enableSFBOpt) {
    int_iss_unit.io.pred_wakeup_port.valid := pred_wakeup.valid
    int_iss_unit.io.pred_wakeup_port.bits := pred_wakeup.bits.uop.pdst
  }


  // ----------------------------------------------------------------
  // Connect the wakeup ports to the busy tables in the rename stages

  for ((renport, intport) <- rename_stage.io.wakeups zip int_ren_wakeups) {
    renport <> intport
  }
  if (usingFPU) {
    for ((renport, fpport) <- fp_rename_stage.io.wakeups zip fp_pipeline.io.wakeups) {
       renport <> fpport
    }
  }
  if (enableSFBOpt) {
    pred_rename_stage.io.wakeups(0) := pred_wakeup
  } else {
    pred_rename_stage.io.wakeups := DontCare
  }

  // If we issue loads back-to-back endlessly (probably because we are executing some tight loop)
  // the store buffer will never drain, breaking the memory-model forward-progress guarantee
  // If we see a large number of loads saturate the LSU, pause for a cycle to let a store drain
  val loads_saturating = (mem_iss_unit.io.iss_valids(0) && mem_iss_unit.io.iss_uops(0).uses_ldq)
  val saturating_loads_counter = RegInit(0.U(5.W))
  when (loads_saturating) { saturating_loads_counter := saturating_loads_counter + 1.U }
  .otherwise { saturating_loads_counter := 0.U }
  val pause_mem = RegNext(loads_saturating) && saturating_loads_counter === ~(0.U(5.W))

  var iss_idx = 0
  var int_iss_cnt = 0
  var mem_iss_cnt = 0
  for (w <- 0 until exe_units.length) {
    var fu_types = exe_units(w).io.fu_types
    val exe_unit = exe_units(w)
    if (exe_unit.readsIrf) {
      if (exe_unit.supportedFuncUnits.muld) {
        // Supress just-issued divides from issuing back-to-back, since it's an iterative divider.
        // But it takes a cycle to get to the Exe stage, so it can't tell us it is busy yet.
        val idiv_issued = iss_valids(iss_idx) && iss_uops(iss_idx).fu_code_is(FU_DIV)
        fu_types = fu_types & RegNext(~Mux(idiv_issued, FU_DIV, 0.U))
      }

      if (exe_unit.hasMem) {
        iss_valids(iss_idx) := mem_iss_unit.io.iss_valids(mem_iss_cnt)
        iss_uops(iss_idx)   := mem_iss_unit.io.iss_uops(mem_iss_cnt)
        mem_iss_unit.io.fu_types(mem_iss_cnt) := Mux(pause_mem, 0.U, fu_types)
        mem_iss_cnt += 1
      } else {
        iss_valids(iss_idx) := int_iss_unit.io.iss_valids(int_iss_cnt)
        iss_uops(iss_idx)   := int_iss_unit.io.iss_uops(int_iss_cnt)
        int_iss_unit.io.fu_types(int_iss_cnt) := fu_types
        int_iss_cnt += 1
      }
      iss_idx += 1
    }
  }
  require(iss_idx == exe_units.numIrfReaders)

  issue_units.map(_.io.tsc_reg := debug_tsc_reg)
  issue_units.map(_.io.brupdate := brupdate)
  issue_units.map(_.io.flush_pipeline := RegNext(rob.io.flush.valid))

  // Load-hit Misspeculations
  require (mem_iss_unit.issueWidth <= 2)
  issue_units.map(_.io.ld_miss := io.lsu.ld_miss)

  mem_units.map(u => u.io.com_exception := RegNext(rob.io.flush.valid))

  // Wakeup (Issue & Writeback)
  for {
    iu <- issue_units
    (issport, wakeup) <- iu.io.wakeup_ports zip int_iss_wakeups
  }{
    issport.valid := wakeup.valid
    issport.bits.pdst := wakeup.bits.uop.pdst
    issport.bits.poisoned := wakeup.bits.uop.iw_p1_poisoned || wakeup.bits.uop.iw_p2_poisoned

    require (iu.io.wakeup_ports.length == int_iss_wakeups.length)
  }

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Register Read Stage ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  // Register Read <- Issue (rrd <- iss)
  iregister_read.io.rf_read_ports <> iregfile.io.read_ports
  iregister_read.io.prf_read_ports := DontCare
  if (enableSFBOpt) {
    iregister_read.io.prf_read_ports <> pregfile.io.read_ports
  }

  for (w <- 0 until exe_units.numIrfReaders) {
    iregister_read.io.iss_valids(w) :=
      iss_valids(w) && !(io.lsu.ld_miss && (iss_uops(w).iw_p1_poisoned || iss_uops(w).iw_p2_poisoned))
  }
  iregister_read.io.iss_uops := iss_uops
  iregister_read.io.iss_uops map { u => u.iw_p1_poisoned := false.B; u.iw_p2_poisoned := false.B }

  iregister_read.io.brupdate := brupdate
  iregister_read.io.kill   := RegNext(rob.io.flush.valid)

  iregister_read.io.bypass := bypasses
  iregister_read.io.pred_bypass := pred_bypasses

  //-------------------------------------------------------------
  // Privileged Co-processor 0 Register File
  // Note: Normally this would be bad in that I'm writing state before
  // committing, so to get this to work I stall the entire pipeline for
  // CSR instructions so I never speculate these instructions.

  val csr_exe_unit = exe_units.csr_unit

  // for critical path reasons, we aren't zero'ing this out if resp is not valid
  val csr_rw_cmd = csr_exe_unit.io.iresp.bits.uop.ctrl.csr_cmd
  val wb_wdata = csr_exe_unit.io.iresp.bits.data

  csr.io.rw.addr        := csr_exe_unit.io.iresp.bits.uop.csr_addr
  csr.io.rw.cmd         := freechips.rocketchip.rocket.CSR.maskCmd(csr_exe_unit.io.iresp.valid, csr_rw_cmd)
  csr.io.rw.wdata       := wb_wdata

  rob.io.csr_replay.valid := csr_exe_unit.io.iresp.valid && csr.io.rw_stall
  rob.io.csr_replay.bits.uop := csr_exe_unit.io.iresp.bits.uop
  rob.io.csr_replay.bits.cause := MINI_EXCEPTION_CSR_REPLAY
  rob.io.csr_replay.bits.badvaddr := DontCare

  // Extra I/O
  // Delay retire/exception 1 cycle
  csr.io.retire    := RegNext(PopCount(rob.io.commit.arch_valids.asUInt))
  ////Enable_Sample_Support: overflow maxevent will not cause exception to kernel
  val isOverFlowExcpt = RegNext(rob.io.com_xcpt.valid && rob.io.com_xcpt.bits.cause === Cause_OverFlow) && overflow_event
  when (isOverFlowExcpt) {
    when (exitFuncAddr =/= 0.U) { // when this is a Event Overflow exception, disable this exception
      csr.io.exception := false.B
      csr.io.cause     := Causes.illegal_instruction.U
    }
    .otherwise {    // when exitFuncAddr, we set the cause be a user_ecall reason, which kernel will execute the ecall processor
      csr.io.exception := true.B
      csr.io.cause     := Causes.user_ecall.U
    }
    userExitPC := csr.io.pc   //record the exception pc for using
  } 
  .otherwise {  // the old setting
    csr.io.exception := RegNext(rob.io.com_xcpt.valid)
    csr.io.cause     := RegNext(rob.io.com_xcpt.bits.cause)
  }
  // csr.io.pc used for setting EPC during exception or CSR.io.trace.

  csr.io.pc        := (boom.v3.util.AlignPCToBoundary(io.ifu.get_pc(0).com_pc, icBlockBytes)
                     + RegNext(rob.io.com_xcpt.bits.pc_lob)
                     - Mux(RegNext(rob.io.com_xcpt.bits.edge_inst), 2.U, 0.U))
  // Cause not valid for for CALL or BREAKPOINTs (CSRFile will override it).
  // csr.io.cause     := RegNext(rob.io.com_xcpt.bits.cause)
  csr.io.ungated_clock := clock

  val tval_valid = csr.io.exception &&
    csr.io.cause.isOneOf(
      //Causes.illegal_instruction.U, we currently only write 0x0 for illegal instructions
      Causes.breakpoint.U,
      Causes.misaligned_load.U,
      Causes.misaligned_store.U,
      Causes.load_access.U,
      Causes.store_access.U,
      Causes.fetch_access.U,
      Causes.load_page_fault.U,
      Causes.store_page_fault.U,
      Causes.fetch_page_fault.U)

  csr.io.tval := Mux(tval_valid,
    RegNext(encodeVirtualAddress(rob.io.com_xcpt.bits.badvaddr, rob.io.com_xcpt.bits.badvaddr)), 0.U)

  // TODO move this function to some central location (since this is used elsewhere).
  def encodeVirtualAddress(a0: UInt, ea: UInt) =
    if (vaddrBitsExtended == vaddrBits) {
      ea
    } else {
      // Efficient means to compress 64-bit VA into vaddrBits+1 bits.
      // (VA is bad if VA(vaddrBits) != VA(vaddrBits-1)).
      val a = a0.asSInt >> vaddrBits
      val msb = Mux(a === 0.S || a === -1.S, ea(vaddrBits), !ea(vaddrBits-1))
      Cat(msb, ea(vaddrBits-1,0))
    }

  // reading requires serializing the entire pipeline
  csr.io.fcsr_flags.valid := rob.io.commit.fflags.valid
  csr.io.fcsr_flags.bits  := rob.io.commit.fflags.bits
  csr.io.set_fs_dirty.get := rob.io.commit.fflags.valid

  exe_units.withFilter(_.hasFcsr).map(_.io.fcsr_rm := csr.io.fcsr_rm)
  io.fcsr_rm := csr.io.fcsr_rm

  if (usingFPU) {
    fp_pipeline.io.fcsr_rm := csr.io.fcsr_rm
  }

  csr.io.hartid := io.hartid
  csr.io.interrupts := io.interrupts

  // we do not support the H-extension
  csr.io.htval := DontCare
  csr.io.gva := DontCare

// TODO can we add this back in, but handle reset properly and save us
//      the mux above on csr.io.rw.cmd?
//   assert (!(csr_rw_cmd =/= rocket.CSR.N && !exe_units(0).io.resp(0).valid),
//   "CSRFile is being written to spuriously.")

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Execute Stage ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  iss_idx = 0
  var bypass_idx = 0
  for (w <- 0 until exe_units.length) {
    val exe_unit = exe_units(w)
    if (exe_unit.readsIrf) {
      exe_unit.io.req <> iregister_read.io.exe_reqs(iss_idx)

      //Enable_PerfCounter_Support: get counter value and send to execution.req.rs1_data
      val uop = exe_unit.io.req.bits.uop
      when (uop.readCounter && uop.ldst =/= 0.U) {
        exe_unit.io.req.bits.rs1_data := event_counters.io.read_data(iss_idx)
      }

      if (exe_unit.bypassable) {
        for (i <- 0 until exe_unit.numBypassStages) {
          bypasses(bypass_idx) := exe_unit.io.bypass(i)
          bypass_idx += 1
        }
      }

      ////Enable_Sample_Support: read special registers data from regfile
      val rrd_uop = iregister_read.io.exe_reqs(iss_idx).bits.uop
      when (rrd_uop.ucsrInst && rrd_uop.ldst === 0.U ) {
        val tag = rrd_uop.inst(31, 20)
        val rs1_data = iregister_read.io.exe_reqs(iss_idx).bits.rs1_data
        switch (tag){
          is (SetUCSR_ProcTag)       { procTag       := rs1_data(31, 0) }
          is (SetUCSR_SampleFuncAddr)  { exitFuncAddr  := rs1_data }
          is (SetUCSR_MaxPriv)       { pfc_maxPriv   := rs1_data(1,0) }
          is (SetUCSR_Temp1)         { tempReg1      := rs1_data }
          is (SetUCSR_Temp2)         { tempReg2      := rs1_data }
          is (SetUCSR_Temp3)         { tempReg3      := rs1_data }
          is (SetUCSR_Temp4)         { tempReg4      := rs1_data }
          is (SetUCSR_EventSel)      { sampleEventSel  := rs1_data(2,0) }
          is (SetUCSR_PfcEnable)     { pfc_enable      := rs1_data(0,0) }
          is (SetUCSR_SampleHappen)  { sampleHappen    := rs1_data(31,0) }
          is (SetUCSR_WarmupInst)    { 
            warmupInstNum := rs1_data 
            nowWarmupInsts := 0.U 
          }
          is (SetUCSR_MaxEventNum)  { 
            maxEventNum := rs1_data
            nowEventNum := 0.U
          }
        }
        // printf("set event value, pc: 0x%x, inst: 0x%x, tag: %d, value: 0x%x\n", rrd_uop.debug_pc, rrd_uop.inst, tag, rs1_data)
      }

      iss_idx += 1
    }
  }
  require (bypass_idx == exe_units.numTotalBypassPorts)
  for (i <- 0 until jmp_unit.numBypassStages) {
    pred_bypasses(i) := jmp_unit.io.bypass(i)
  }

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Load/Store Unit ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  // enqueue basic load/store info in Decode
  for (w <- 0 until coreWidth) {
    io.lsu.dis_uops(w).valid := dis_fire(w)
    io.lsu.dis_uops(w).bits  := dis_uops(w)
  }

  // tell LSU about committing loads and stores to clear entries
  io.lsu.commit                  := rob.io.commit

  // tell LSU that it should fire a load that waits for the rob to clear
  io.lsu.commit_load_at_rob_head := rob.io.com_load_is_at_rob_head

  //com_xcpt.valid comes too early, will fight against a branch that resolves same cycle as an exception
  io.lsu.exception := RegNext(rob.io.flush.valid)

  // Handle Branch Mispeculations
  io.lsu.brupdate := brupdate
  io.lsu.rob_head_idx := rob.io.rob_head_idx
  io.lsu.rob_pnr_idx  := rob.io.rob_pnr_idx

  io.lsu.tsc_reg := debug_tsc_reg


  if (usingFPU) {
    io.lsu.fp_stdata <> fp_pipeline.io.to_sdq
  }

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Writeback Stage ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  var w_cnt = 1
  iregfile.io.write_ports(0) := WritePort(ll_wbarb.io.out, ipregSz, xLen, RT_FIX)
  ll_wbarb.io.in(0) <> mem_resps(0)
  assert (ll_wbarb.io.in(0).ready) // never backpressure the memory unit.
  for (i <- 1 until memWidth) {
    iregfile.io.write_ports(w_cnt) := WritePort(mem_resps(i), ipregSz, xLen, RT_FIX)
    w_cnt += 1
  }

  for (i <- 0 until exe_units.length) {
    if (exe_units(i).writesIrf) {
      val wbresp = exe_units(i).io.iresp
      val wbpdst = wbresp.bits.uop.pdst
      val wbdata = wbresp.bits.data

      def wbIsValid(rtype: UInt) =
        wbresp.valid && wbresp.bits.uop.rf_wen && wbresp.bits.uop.dst_rtype === rtype
      val wbReadsCSR = wbresp.bits.uop.ctrl.csr_cmd =/= freechips.rocketchip.rocket.CSR.N

      iregfile.io.write_ports(w_cnt).valid     := wbIsValid(RT_FIX)
      iregfile.io.write_ports(w_cnt).bits.addr := wbpdst
      wbresp.ready := true.B
      if (exe_units(i).hasCSR) {
        iregfile.io.write_ports(w_cnt).bits.data := Mux(wbReadsCSR, csr.io.rw.rdata, wbdata)
      } else {
        iregfile.io.write_ports(w_cnt).bits.data := wbdata
      }

      //Enable_Sample_Support: write special registers data to regfile
      when (wbresp.bits.uop.ucsrInst && wbresp.bits.uop.ldst =/= 0.U ) {
        val tag = wbresp.bits.uop.inst(31, 20)
        switch (tag) {
          is (GetUCSR_ProcTag)   { iregfile.io.write_ports(w_cnt).bits.data := Cat(0.U(32.W), procTag) }
          is (GetUCSR_ExitNPC)   { iregfile.io.write_ports(w_cnt).bits.data := userExitPC }
          is (GetUCSR_Temp1)     { iregfile.io.write_ports(w_cnt).bits.data := tempReg1 }
          is (GetUCSR_Temp2)     { iregfile.io.write_ports(w_cnt).bits.data := tempReg2 }
          is (GetUCSR_Temp3)     { iregfile.io.write_ports(w_cnt).bits.data := tempReg3 }
          is (GetUCSR_Temp4)     { iregfile.io.write_ports(w_cnt).bits.data := tempReg4 }
          is (GetUCSR_SampleHappen) { iregfile.io.write_ports(w_cnt).bits.data := Cat(0.U(32.W), sampleHappen) }

          is (GetUCSR_SampleFuncAddr) { iregfile.io.write_ports(w_cnt).bits.data := exitFuncAddr }
          is (GetUCSR_MaxEvents)    { iregfile.io.write_ports(w_cnt).bits.data := maxEventNum }
          is (GetUCSR_NowEvents)    { iregfile.io.write_ports(w_cnt).bits.data := nowEventNum }
          is (GetUCSR_EventSel)     { iregfile.io.write_ports(w_cnt).bits.data := Cat(0.U(61.W), sampleEventSel) }
          is (GetUCSR_Maxpriv)   { iregfile.io.write_ports(w_cnt).bits.data := Cat(0.U(62.W), pfc_maxPriv) }
        }
      }

      assert (!wbIsValid(RT_FLT), "[fppipeline] An FP writeback is being attempted to the Int Regfile.")

      assert (!(wbresp.valid &&
        !wbresp.bits.uop.rf_wen &&
        wbresp.bits.uop.dst_rtype === RT_FIX),
        "[fppipeline] An Int writeback is being attempted with rf_wen disabled.")

      assert (!(wbresp.valid &&
        wbresp.bits.uop.rf_wen &&
        wbresp.bits.uop.dst_rtype =/= RT_FIX),
        "[fppipeline] writeback being attempted to Int RF with dst != Int type exe_units("+i+").iresp")
      w_cnt += 1
    }
  }
  require(w_cnt == iregfile.io.write_ports.length)

  if (enableSFBOpt) {
    pregfile.io.write_ports(0).valid     := jmp_unit.io.iresp.valid && jmp_unit.io.iresp.bits.uop.is_sfb_br
    pregfile.io.write_ports(0).bits.addr := jmp_unit.io.iresp.bits.uop.pdst
    pregfile.io.write_ports(0).bits.data := jmp_unit.io.iresp.bits.data
  }

  if (usingFPU) {
    // Connect IFPU
    fp_pipeline.io.from_int  <> exe_units.ifpu_unit.io.ll_fresp
    // Connect FPIU
    ll_wbarb.io.in(1)        <> fp_pipeline.io.to_int
    // Connect FLDs
    fp_pipeline.io.ll_wports <> exe_units.memory_units.map(_.io.ll_fresp).toSeq
  }
  if (usingRoCC) {
    require(usingFPU)
    ll_wbarb.io.in(2)       <> exe_units.rocc_unit.io.ll_iresp
  }

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Commit Stage ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  // Writeback
  // ---------
  // First connect the ll_wport
  val ll_uop = ll_wbarb.io.out.bits.uop
  rob.io.wb_resps(0).valid  := ll_wbarb.io.out.valid && !(ll_uop.uses_stq && !ll_uop.is_amo)
  rob.io.wb_resps(0).bits   <> ll_wbarb.io.out.bits
  rob.io.debug_wb_valids(0) := ll_wbarb.io.out.valid && ll_uop.dst_rtype =/= RT_X
  rob.io.debug_wb_wdata(0)  := ll_wbarb.io.out.bits.data
  var cnt = 1
  for (i <- 1 until memWidth) {
    val mem_uop = mem_resps(i).bits.uop
    rob.io.wb_resps(cnt).valid := mem_resps(i).valid && !(mem_uop.uses_stq && !mem_uop.is_amo)
    rob.io.wb_resps(cnt).bits  := mem_resps(i).bits
    rob.io.debug_wb_valids(cnt) := mem_resps(i).valid && mem_uop.dst_rtype =/= RT_X
    rob.io.debug_wb_wdata(cnt)  := mem_resps(i).bits.data
    cnt += 1
  }
  var f_cnt = 0 // rob fflags port index
  for (eu <- exe_units) {
    if (eu.writesIrf)
    {
      val resp   = eu.io.iresp
      val wb_uop = resp.bits.uop
      val data   = resp.bits.data

      rob.io.wb_resps(cnt).valid := resp.valid && !(wb_uop.uses_stq && !wb_uop.is_amo)
      rob.io.wb_resps(cnt).bits  <> resp.bits
      rob.io.debug_wb_valids(cnt) := resp.valid && wb_uop.rf_wen && wb_uop.dst_rtype === RT_FIX
      if (eu.hasFFlags) {
        rob.io.fflags(f_cnt) <> resp.bits.fflags
        f_cnt += 1
      }
      if (eu.hasCSR) {
        rob.io.debug_wb_wdata(cnt) := Mux(wb_uop.ctrl.csr_cmd =/= freechips.rocketchip.rocket.CSR.N,
          csr.io.rw.rdata,
          data)
      } else {
        rob.io.debug_wb_wdata(cnt) := data
      }
      cnt += 1
    }
  }

  require(cnt == numIrfWritePorts)
  if (usingFPU) {
    for ((wdata, wakeup) <- fp_pipeline.io.debug_wb_wdata zip fp_pipeline.io.wakeups) {
      rob.io.wb_resps(cnt) <> wakeup
      rob.io.fflags(f_cnt) <> wakeup.bits.fflags
      rob.io.debug_wb_valids(cnt) := wakeup.valid
      rob.io.debug_wb_wdata(cnt) := wdata
      cnt += 1
      f_cnt += 1

      assert (!(wakeup.valid && wakeup.bits.uop.dst_rtype =/= RT_FLT),
        "[core] FP wakeup does not write back to a FP register.")

      assert (!(wakeup.valid && !wakeup.bits.uop.fp_val),
        "[core] FP wakeup does not involve an FP instruction.")
    }
  }

  require (cnt == rob.numWakeupPorts)
  require (f_cnt == rob.numFpuPorts)

  // branch resolution
  rob.io.brupdate <> brupdate

  exe_units.map(u => u.io.status := csr.io.status)
  if (usingFPU)
    fp_pipeline.io.status := csr.io.status

  // Connect breakpoint info to memaddrcalcunit
  for (i <- 0 until memWidth) {
    mem_units(i).io.status   := csr.io.status
    mem_units(i).io.bp       := csr.io.bp
    mem_units(i).io.mcontext := csr.io.mcontext
    mem_units(i).io.scontext := csr.io.scontext
  }

  // LSU <> ROB
  rob.io.lsu_clr_bsy    := io.lsu.clr_bsy
  rob.io.lsu_clr_unsafe := io.lsu.clr_unsafe
  rob.io.lxcpt          <> io.lsu.lxcpt

  assert (!(csr.io.singleStep), "[core] single-step is unsupported.")


  //-------------------------------------------------------------
  // **** Flush Pipeline ****
  //-------------------------------------------------------------
  // flush on exceptions, miniexeptions, and after some special instructions

  if (usingFPU) {
    fp_pipeline.io.flush_pipeline := RegNext(rob.io.flush.valid)
  }

  for (w <- 0 until exe_units.length) {
    exe_units(w).io.req.bits.kill := RegNext(rob.io.flush.valid)
  }

  assert (!(rob.io.com_xcpt.valid && !rob.io.flush.valid),
    "[core] exception occurred, but pipeline flush signal not set!")

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Outputs to the External World ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  // detect pipeline freezes and throw error
  val idle_cycles = freechips.rocketchip.util.WideCounter(32)
  when (rob.io.commit.valids.asUInt.orR ||
        csr.io.csr_stall ||
        io.rocc.busy ||
        reset.asBool) {
    idle_cycles := 0.U
  }
  assert (!(idle_cycles.value(13)), "Pipeline has hung.")

  if (usingFPU) {
    fp_pipeline.io.debug_tsc_reg := debug_tsc_reg
  }

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Handle Cycle-by-Cycle Printouts ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------


  if (IN_SIMULATION) {
    // Add a free-running cycle counter for simulation
    val (cycleCount, _) = Counter(true.B, Int.MaxValue)

    val reset_counter_period = PlusArg("reset-event-counters", 0, "Reset event counters every N committed instructions (0 to disable)", 32)
    val print_count = RegInit(0.U(32.W))
    val print_count_write = WireDefault(print_count)
    print_count := print_count_write
    when (startCounter && reset_counter_period > 0.U) {
      print_count_write := print_count + PopCount(rob.io.commit.arch_valids.asUInt)
      when (print_count_write >= reset_counter_period) {
        print_count := 0.U
        event_counters.io.reset_counter := true.B
        printf("**** Event Counters Reset ****\n")
      }
    }

    val cmd_ras_printf = PlusArg("ras-printf", 0, "print RAS updates", 1)
    when (cmd_ras_printf(0) && brupdate.b2.mispredict && !RegNext(rob.io.flush.valid)) {
      val mis_uop = brupdate.b2.uop
      val mis_is_call = (mis_uop.is_jalr || mis_uop.is_jal) && ((mis_uop.ldst === 1.U) || (mis_uop.ldst === 5.U)) // c.jal 在 RV64 中会被译码为 c.addiw
      val mis_is_ret  = mis_uop.is_jalr && ((mis_uop.lrs1 === 1.U) || (mis_uop.lrs1 === 5.U)) && !mis_is_call
      printf("**** %d MISPREDICT ftq=%x pc=0x%x target=0x%x is_call=%d is_ret=%d ****\n",
        cycleCount,
        mis_uop.ftq_idx,
        Sext(mis_uop.debug_pc(vaddrBits-1,0), xLen),
        Sext(io.ifu.redirect_pc(vaddrBits-1,0), xLen),
        mis_is_call,
        mis_is_ret)
    }
    // 打印出 commit call/ret，方便对照
    when (cmd_ras_printf(0) && (com_is_call.reduce(_ || _) || com_is_ret.reduce(_ || _))) {
      for (w <- 0 until coreWidth) {
        when (com_is_call(w) || com_is_ret(w)) {
          val uop = rob.io.commit.uops(w)
            printf("**** %d COMMIT ftq=%x pc=0x%x com_call=%d com_ret=%d ****\n",
              cycleCount,
              uop.ftq_idx,
              Sext(uop.debug_pc(vaddrBits-1,0), xLen),
              com_is_call(w),
              com_is_ret(w))
        }
      }
    }

    val commit_printf = PlusArg("commit-printf", 0, "print predecode targets", 1)
    when (commit_printf(0)) {
    var new_commit_cnt = 0.U

    for (w <- 0 until coreWidth) {
      val priv = RegNext(csr.io.status.prv) // erets change the privilege. Get the old one

      // To allow for diffs against spike :/
      def printf_inst(uop: MicroOp) = {
        when (uop.is_rvc) {
          printf("(0x%x)", uop.debug_inst(15,0))
        } .otherwise {
          printf("(0x%x)", uop.debug_inst)
        }
      }

      when (rob.io.commit.valids(w)) {
        printf("[%d] %d 0x%x ",
          cycleCount, priv,
          Sext(rob.io.commit.uops(w).debug_pc(vaddrBits-1,0), xLen))
        printf_inst(rob.io.commit.uops(w))
        when (!rob.io.commit.arch_valids(w)) {
          printf(" (not arch) ")
        }
        when (rob.io.commit.uops(w).dst_rtype === RT_FIX && rob.io.commit.uops(w).ldst =/= 0.U) {
          printf(" x%d 0x%x, ftq_idx 0x%x\n",
            rob.io.commit.uops(w).ldst,
            rob.io.commit.debug_wdata(w),
            rob.io.commit.uops(w).ftq_idx)
        } .elsewhen (rob.io.commit.uops(w).dst_rtype === RT_FLT) {
          printf(" f%d 0x%x, ftq_idx 0x%x\n",
            rob.io.commit.uops(w).ldst,
            rob.io.commit.debug_wdata(w),
            rob.io.commit.uops(w).ftq_idx)
        } .otherwise {
          printf(" ftq_idx 0x%x\n", rob.io.commit.uops(w).ftq_idx)
        }
      }
    }}
  } else if (BRANCH_PRINTF) {
    val debug_ghist = RegInit(0.U(globalHistoryLength.W))
    when (rob.io.flush.valid && FlushTypes.useCsrEvec(rob.io.flush.bits.flush_typ)) {
      debug_ghist := 0.U
    }

    var new_ghist = debug_ghist

    for (w <- 0 until coreWidth) {
      when (rob.io.commit.arch_valids(w) &&
        (rob.io.commit.uops(w).is_br || rob.io.commit.uops(w).is_jal || rob.io.commit.uops(w).is_jalr)) {
        // for (i <- 0 until globalHistoryLength) {
        //   printf("%x", new_ghist(globalHistoryLength-i-1))
        // }
        // printf("\n")
        printf("%x %x %x %x %x %x\n",
          rob.io.commit.uops(w).debug_fsrc, rob.io.commit.uops(w).taken,
          rob.io.commit.uops(w).is_br, rob.io.commit.uops(w).is_jal,
          rob.io.commit.uops(w).is_jalr, Sext(rob.io.commit.uops(w).debug_pc(vaddrBits-1,0), xLen))

      }
      new_ghist = Mux(rob.io.commit.arch_valids(w) && rob.io.commit.uops(w).is_br,
        Mux(rob.io.commit.uops(w).taken, new_ghist << 1 | 1.U(1.W), new_ghist << 1),
        new_ghist)
    }
    debug_ghist := new_ghist
  }

  // TODO: Does anyone want this debugging functionality?
  val coreMonitorBundle = Wire(new CoreMonitorBundle(xLen, fLen))
  coreMonitorBundle := DontCare
  coreMonitorBundle.clock  := clock
  coreMonitorBundle.reset  := reset


  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // Page Table Walker

  io.ptw.ptbr       := csr.io.ptbr
  io.ptw.status     := csr.io.status
  io.ptw.pmp        := csr.io.pmp
  io.ptw.sfence     := io.ifu.sfence

  //-------------------------------------------------------------
  //-------------------------------------------------------------

  io.rocc := DontCare
  io.rocc.exception := csr.io.exception && csr.io.status.xs.orR
  io.rocc.csrs <> csr.io.roccCSRs
  if (usingRoCC) {
    exe_units.rocc_unit.io.rocc.rocc         <> io.rocc
    exe_units.rocc_unit.io.rocc.dis_uops     := dis_uops
    exe_units.rocc_unit.io.rocc.rob_head_idx := rob.io.rob_head_idx
    exe_units.rocc_unit.io.rocc.rob_pnr_idx  := rob.io.rob_pnr_idx
    exe_units.rocc_unit.io.com_exception     := rob.io.flush.valid
    exe_units.rocc_unit.io.status            := csr.io.status

    for (w <- 0 until coreWidth) {
      exe_units.rocc_unit.io.rocc.dis_rocc_vals(w) := (
        dis_fire(w) &&
        dis_uops(w).uopc === uopROCC &&
        !dis_uops(w).exception
      )
    }
  }

  io.trace := DontCare
  io.trace.time := csr.io.time
  io.trace.insns map (t => t.valid := false.B)
  io.trace.custom.get.asInstanceOf[BoomTraceBundle].rob_empty := rob.io.empty

  if (trace) {
    for (w <- 0 until coreWidth) {
      // Delay the trace so we have a cycle to pull PCs out of the FTQ
      io.trace.insns(w).valid      := RegNext(rob.io.commit.arch_valids(w))

      // Recalculate the PC
      io.ifu.debug_ftq_idx(w) := rob.io.commit.uops(w).ftq_idx
      val iaddr = (AlignPCToBoundary(io.ifu.debug_fetch_pc(w), icBlockBytes)
                   + RegNext(rob.io.commit.uops(w).pc_lob)
                   - Mux(RegNext(rob.io.commit.uops(w).edge_inst), 2.U, 0.U))(vaddrBits-1,0)
      io.trace.insns(w).iaddr      := Sext(iaddr, xLen)

      def getInst(uop: MicroOp, inst: UInt): UInt = {
        Mux(uop.is_rvc, Cat(0.U(16.W), inst(15,0)), inst)
      }

      def getWdata(uop: MicroOp, wdata: UInt): UInt = {
        Mux((uop.dst_rtype === RT_FIX && uop.ldst =/= 0.U) || (uop.dst_rtype === RT_FLT), wdata, 0.U(xLen.W))
      }

      // use debug_insts instead of uop.debug_inst to use the rob's debug_inst_mem
      // note: rob.debug_insts comes 1 cycle later
      io.trace.insns(w).insn       := getInst(RegNext(rob.io.commit.uops(w)), rob.io.commit.debug_insts(w))
      io.trace.insns(w).wdata.map { _ := RegNext(getWdata(rob.io.commit.uops(w), rob.io.commit.debug_wdata(w))) }

      // Comment out this assert because it blows up FPGA synth-asserts
      // This tests correctedness of the debug_inst mem
      // when (RegNext(rob.io.commit.valids(w))) {
      //   assert(rob.io.commit.debug_insts(w) === RegNext(rob.io.commit.uops(w).debug_inst))
      // }
      // This tests correctedness of recovering pcs through ftq debug ports
      // when (RegNext(rob.io.commit.valids(w))) {
      //   assert(Sext(io.trace.insns(w).iaddr, xLen) ===
      //     RegNext(Sext(rob.io.commit.uops(w).debug_pc(vaddrBits-1,0), xLen)))
      // }

      // These csr signals do not exactly match up with the ROB commit signals.
      io.trace.insns(w).priv       := RegNext(Cat(RegNext(csr.io.status.debug), csr.io.status.prv))
      // Can determine if it is an interrupt or not based on the MSB of the cause
      io.trace.insns(w).exception  := RegNext(rob.io.com_xcpt.valid && !rob.io.com_xcpt.bits.cause(xLen - 1)) && (w == 0).B
      io.trace.insns(w).interrupt  := RegNext(rob.io.com_xcpt.valid && rob.io.com_xcpt.bits.cause(xLen - 1)) && (w == 0).B
      io.trace.insns(w).cause      := RegNext(rob.io.com_xcpt.bits.cause)
      io.trace.insns(w).tval       := RegNext(csr.io.tval)
    }
    dontTouch(io.trace)
  } else {
    io.ifu.debug_ftq_idx := DontCare
  }
}

//******************************************************************************
// Copyright (c) 2015 - 2019, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Fetch Target Queue (FTQ)
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
// Each entry in the FTQ holds the fetch address and branch prediction snapshot state.
//
// TODO:
// * reduce port counts.

package boom.v3.ifu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.{Parameters}
import freechips.rocketchip.util.{Str, PlusArg}
import freechips.rocketchip.tile.TileKey

import boom.v3.common._
import boom.v3.exu._
import boom.v3.util._

/**
 * FTQ Parameters used in configurations
 *
 * @param nEntries # of entries in the FTQ
 */
case class FtqParameters(
  nEntries: Int = 16
)

/**
 * Bundle to add to the FTQ RAM and to be used as the pass in IO
 */
class FTQBundle(implicit p: Parameters) extends BoomBundle
  with HasBoomFrontendParameters
{
  // // TODO compress out high-order bits
  // val fetch_pc  = UInt(vaddrBitsExtended.W)
  // IDX of instruction that was predicted taken, if any
  val cfi_idx   = Valid(UInt(log2Ceil(fetchWidth).W))
  // Was the CFI in this bundle found to be taken? or not
  val cfi_taken = Bool()
  // Was this CFI mispredicted by the branch prediction pipeline?
  val cfi_mispredicted = Bool()
  // What type of CFI was taken out of this bundle
  val cfi_type = UInt(CFI_SZ.W)
  // mask of branches which were visible in this fetch bundle
  val br_mask   = UInt(fetchWidth.W)
  // This CFI is likely a CALL
  val cfi_is_call   = Bool()
  // This CFI is likely a RET
  val cfi_is_ret    = Bool()
  // Is the NPC after the CFI +4 or +2
  val cfi_npc_plus4 = Bool()
  // What was the top of the RAS that this bundle saw?
  val ras_top = UInt(vaddrBitsExtended.W)
  val ras_idx = UInt(log2Ceil(nRasEntries).W)

  // btb 将非分支指令或者非指令预测为了分支指令
  val btb_mispredicts = UInt(fetchWidth.W)

  // fetch packet 的来源
  val ft_tsrc = UInt(BSRC_SZ.W)

  // fetch packet 是否引发了 rob flush
  val has_exp = Bool()

  // Which bank did this start from?
  val start_bank = UInt(1.W)

  // // Metadata for the branch predictor
  // val bpd_meta = Vec(nBanks, UInt(bpdMaxMetaLength.W))
}

// 它保存仅在预译码阶段使用的预测信息
class FTQPredsInfo(implicit p: Parameters) extends BoomBundle
  with HasBoomFrontendParameters
{
  // 预测条件跳转是否 taken
  val br_taken          = UInt(fetchWidth.W)
  // 第一个无条件跳转指令的目标地址
  val jal_target        = UInt(vaddrBitsExtended.W)
  val tsrc              = UInt(BSRC_SZ.W)
  val fsrc              = UInt(BSRC_SZ.W)
  //  BTB 命中信息
  val btb_hits          = UInt(fetchWidth.W)
  val ghist_update_type = UInt(2.W)
}

class FTQToPredecodeInfo(implicit p: Parameters) extends BoomBundle
  with HasBoomFrontendParameters
{
  val preds_info      = new FTQPredsInfo
  val pc_debug        = UInt(vaddrBitsExtended.W)
  val ghist           = new GlobalHistory
  val ras_top         = UInt(vaddrBitsExtended.W)
  val ras_idx         = UInt(log2Ceil(nRasEntries).W)
  val check_target    = UInt(vaddrBitsExtended.W)
  val bpd_target      = UInt(vaddrBitsExtended.W)
  
  // TODO：是否要加上 cfi_is_call 和 cfi_is_ret 来修正 ras？
  // val cfi_is_call = Bool()
  // val cfi_is_ret  = Bool()
}

/**
 * IO to provide a port for a FunctionalUnit to get the PC of an instruction.
 * And for JALRs, the PC of the next instruction.
 */
class GetPCFromFtqIO(implicit p: Parameters) extends BoomBundle
{
  val ftq_idx   = Input(UInt(log2Ceil(ftqSz).W))

  val entry     = Output(new FTQBundle)
  val ghist     = Output(new GlobalHistory)

  val pc        = Output(UInt(vaddrBitsExtended.W))
  val com_pc    = Output(UInt(vaddrBitsExtended.W))

  // the next_pc may not be valid (stalled or still being fetched)
  val next_val  = Output(Bool())
  val next_pc   = Output(UInt(vaddrBitsExtended.W))
}

class FTQPtr(entries: Int)(implicit p: Parameters) extends CircularQueuePtr[FTQPtr](entries)
{
  // 好像在执行构造函数的时候 trait 内部的代码还未执行，因此这里拿不到 ftqSz
  def this()(implicit p: Parameters) = this(p(TileKey).core.asInstanceOf[BoomCoreParams].ftq.nEntries)
  require(ftqSz == entries)
}

object FTQPtr {
  def apply(f: Bool, v: UInt)(implicit p: Parameters): FTQPtr = {
    val ptr = Wire(new FTQPtr)
    ptr.flag  := f
    ptr.value := v
    ptr
  }
  def inverse(ptr: FTQPtr)(implicit p: Parameters): FTQPtr =
    apply(!ptr.flag, ptr.value)
}

/**
 * Queue to store the fetch PC and other relevant branch predictor signals that are inflight in the
 * processor.
 *
 * @param num_entries # of entries in the FTQ
 */
class FetchTargetQueue(implicit p: Parameters) extends BoomModule
  with HasBoomCoreParameters
  with HasBoomFrontendParameters
  with HasCircularQueuePtrHelper
{
  val num_entries = ftqSz
  private val idx_sz = log2Ceil(num_entries)

  val io = IO(new BoomBundle {
    // f3 preds enq
    val f3_preds_enq     = Flipped(Valid(new F3PredsToFTQ()))
    val f3_ftq_idx_debug = Input(new FTQPtr())
    // 获取 prefetch 的 pc
    val pf_ftq_idx = Input(new FTQPtr())
    val pf_pc = Output(Valid(UInt(vaddrBitsExtended.W)))
    // 获取 ifu 取指 pc
    val ifu_fetch_ftq_idx = Input(new FTQPtr()) 
    val ifu_fetch_pc = Valid(UInt(vaddrBitsExtended.W))
    // 从 ftq 中获取预测信息需要一个周期
    val s2_ftq_idx = Input(new FTQPtr())
    val s3_preds_info = Output(new FTQToPredecodeInfo())
    // predecode 重定向信号
    val predecode_redirect = Input(Valid(new FTQPtr()))
    val merged_f3_target = Input(UInt(vaddrBitsExtended.W))
    // predeocde 信息存入 FTQ
    val predecode_enq = Flipped(Valid(new FetchBundle()))
    // 指示当前队尾指针
    val bpd_commit_ptr = Output(new FTQPtr())
    // TODO: remove this
    val redirect_ftq_idx = Output(new FTQPtr())
    // ROB tells us the youngest committed ftq_idx to remove from FTQ.
    val deq = Flipped(Valid(UInt(idx_sz.W)))
    // 用于 event counter 统计
    val ft_tsrc = Valid(UInt(BSRC_SZ.W))

    // Give PC info to BranchUnit.
    val get_ftq_pc = Vec(2, new GetPCFromFtqIO())


    // Used to regenerate PC for trace port stuff in FireSim
    // Don't tape this out, this blows up the FTQ
    val debug_ftq_idx  = Input(Vec(coreWidth, UInt(log2Ceil(ftqSz).W)))
    val debug_fetch_pc = Output(Vec(coreWidth, UInt(vaddrBitsExtended.W)))

    val redirect = Input(Valid(UInt(idx_sz.W)))
    val redirect_target = Input(UInt(vaddrBitsExtended.W))
    // 区分 rob flush 和 mispredict
    val rob_flush = Input(Bool())
    val rob_flush_pc_lob = Input(UInt(log2Ceil(icBlockBytes).W))

    val brupdate = Input(new BrUpdateInfo)

    val bpdupdate = Output(Valid(new BranchPredictionUpdate))
    val update_miss_counter = Output(Bool())
    val last_commit_cfi_type = Output(UInt(IC_MISS_CAUSE_SZ.W))
    val commit_ic_stall_cycles = Output(UInt(12.W))

    val ras_update = Output(Bool())
    val ras_update_idx = Output(UInt(log2Ceil(nRasEntries).W))
    val ras_update_pc  = Output(UInt(vaddrBitsExtended.W))

  })
  val bpd_commit_ptr               = RegInit(FTQPtr(false.B, 0.U))
  val deq_ptr                      = RegInit(FTQPtr(false.B, 0.U))
  val f3_pred_enq_ptr              = RegInit(FTQPtr(false.B, 0.U))
  val predecode_enq_ptr            = io.predecode_enq.bits.ftq_idx
  val next_predecode_enq_ptr_debug = RegInit(FTQPtr(false.B, 0.U))

  io.bpd_commit_ptr := bpd_commit_ptr
  val full = isFull(predecode_enq_ptr, bpd_commit_ptr)


  val pcs      = Reg(Vec(num_entries, UInt(vaddrBitsExtended.W)))
  val meta     = SyncReadMem(num_entries, Vec(nBanks, UInt(bpdMaxMetaLength.W)))
  val ram      = Reg(Vec(num_entries, new FTQBundle))
  val ghist    = Seq.fill(3) { SyncReadMem(num_entries, new GlobalHistory) }
  val preds_info = SyncReadMem(num_entries, new FTQPredsInfo)

  // 统计每个 fetch packet 的 icache miss stall cycle 数
  val icache_stall_cycles = SyncReadMem(num_entries, UInt(12.W))

  // 处理 f3 preds enq
  val do_f3_preds_enq        = io.f3_preds_enq.valid
  val last_f3_enq_ghist      = Reg(new GlobalHistory)
  val last_f3_enq_ras_idx    = Reg(UInt(log2Ceil(nRasEntries).W))
  val last_f3_enq_ras_top    = Reg(UInt(vaddrBitsExtended.W))
  val last_f3_enq_preds_info = Reg(new FTQPredsInfo)
  val last_f3_enq_target     = Reg(UInt(vaddrBitsExtended.W))
  val new_preds_info         = Wire(new FTQPredsInfo)
  new_preds_info        := DontCare
  
  val first_f3_enq_done = RegInit(false.B)
  when (do_f3_preds_enq) {
    first_f3_enq_done := true.B
  }
  io.pf_pc.valid := io.pf_ftq_idx < f3_pred_enq_ptr || (io.pf_ftq_idx === f3_pred_enq_ptr && first_f3_enq_done)
  io.pf_pc.bits  := Mux(io.pf_ftq_idx === f3_pred_enq_ptr, last_f3_enq_target, pcs(io.pf_ftq_idx.value))

  // 当 fetch idx 和 pred ptr 的 value 相等但 flag 不同时，一定是
  // pred ptr 领先 fetch idx 一圈
  io.ifu_fetch_pc.valid := io.ifu_fetch_ftq_idx < f3_pred_enq_ptr ||
                          (io.ifu_fetch_ftq_idx === f3_pred_enq_ptr && first_f3_enq_done)
  io.ifu_fetch_pc.bits  := Mux(io.ifu_fetch_ftq_idx === f3_pred_enq_ptr,
                          last_f3_enq_target, pcs(io.ifu_fetch_ftq_idx.value))

  when (do_f3_preds_enq) {
    new_preds_info.br_taken    := io.f3_preds_enq.bits.preds_info.br_taken
    new_preds_info.jal_target  := io.f3_preds_enq.bits.preds_info.jal_target
    new_preds_info.btb_hits    := io.f3_preds_enq.bits.preds_info.btb_hits
    new_preds_info.tsrc        := io.f3_preds_enq.bits.fetch_info.tsrc
    new_preds_info.fsrc        := io.f3_preds_enq.bits.fetch_info.fsrc
    new_preds_info.ghist_update_type := io.f3_preds_enq.bits.target_info.f3_pred_ghist_update_type

    preds_info.write(f3_pred_enq_ptr.value, new_preds_info)
    meta.write(f3_pred_enq_ptr.value, io.f3_preds_enq.bits.fetch_info.meta)
    ghist.map(g => g.write(f3_pred_enq_ptr.value, io.f3_preds_enq.bits.fetch_info.ghist))
    pcs(f3_pred_enq_ptr.value) := io.f3_preds_enq.bits.fetch_info.pc

    // 用于 bypass
    last_f3_enq_ghist := io.f3_preds_enq.bits.fetch_info.ghist
    last_f3_enq_ras_idx := io.f3_preds_enq.bits.preds_info.ras_idx
    last_f3_enq_ras_top := io.f3_preds_enq.bits.preds_info.ras_top
    last_f3_enq_preds_info := new_preds_info
    last_f3_enq_target     := io.merged_f3_target

    // TODO: FTQ bundle 的 ras idx 和 ras top 字段会被 f3 preds 和 predecode 都写入，
    // 是否需要把这俩字段单独分离出来？
    val new_preds_info_2 = Wire(new FTQBundle)
    new_preds_info_2           := DontCare
    new_preds_info_2.ras_idx   := io.f3_preds_enq.bits.preds_info.ras_idx
    new_preds_info_2.ras_top   := io.f3_preds_enq.bits.preds_info.ras_top

    ram(f3_pred_enq_ptr.value) := new_preds_info_2

    f3_pred_enq_ptr := f3_pred_enq_ptr + 1.U
  }

  // 发送 preds info 到 predecode
  val s2_next_ftq_idx   = io.s2_ftq_idx + 1.U
  val target_bypass     = s2_next_ftq_idx === f3_pred_enq_ptr && do_f3_preds_enq
  dontTouch(target_bypass)
  dontTouch(s2_next_ftq_idx)
  val s3_ftq_idx        = RegNext(io.s2_ftq_idx)
  val s3_bypass         = RegNext((io.s2_ftq_idx + (!do_f3_preds_enq)) === f3_pred_enq_ptr)
  val s3_ghist          = ghist(2).read(io.s2_ftq_idx.value, true.B)
  val s3_mem_preds_info = preds_info.read(io.s2_ftq_idx.value, true.B)
  val s3_mem_pred_target = RegNext(Mux(target_bypass,
                                io.f3_preds_enq.bits.fetch_info.pc,
                                pcs(s2_next_ftq_idx.value)))
  // 如果要读的 preds info 在同一周期送过来
  when (do_f3_preds_enq && s3_ftq_idx === f3_pred_enq_ptr) {
    io.s3_preds_info.ghist       := io.f3_preds_enq.bits.fetch_info.ghist
    io.s3_preds_info.ras_idx     := io.f3_preds_enq.bits.preds_info.ras_idx
    io.s3_preds_info.ras_top     := io.f3_preds_enq.bits.preds_info.ras_top
    io.s3_preds_info.pc_debug    := io.f3_preds_enq.bits.fetch_info.pc
    io.s3_preds_info.preds_info  := new_preds_info
    // 如果预译码阶段和 f3 阶段预测的是同一个 fetch packet，那么预译码阶段实际应该
    // 重定向 f2 预测
    io.s3_preds_info.check_target := io.f3_preds_enq.bits.target_info.f2_pred_target
    io.s3_preds_info.bpd_target   := io.f3_preds_enq.bits.target_info.f3_pred_target
    io.s3_preds_info.preds_info.ghist_update_type := io.f3_preds_enq.bits.target_info.f2_pred_ghist_update_type
  // 如果要读的 preds info 是上一个写入的
  } .elsewhen(s3_bypass) {
    io.s3_preds_info.ghist       := last_f3_enq_ghist
    io.s3_preds_info.ras_idx     := last_f3_enq_ras_idx
    io.s3_preds_info.ras_top     := last_f3_enq_ras_top
    io.s3_preds_info.pc_debug    := pcs(s3_ftq_idx.value)
    io.s3_preds_info.preds_info  := last_f3_enq_preds_info
    io.s3_preds_info.check_target := last_f3_enq_target
    io.s3_preds_info.bpd_target   := last_f3_enq_target
  // 如果要读的 preds info 写入了，并且它之后还有新的 preds info 写入
  } .otherwise {
    io.s3_preds_info.ghist       := s3_ghist
    io.s3_preds_info.ras_idx     := RegNext(ram(io.s2_ftq_idx.value).ras_idx)
    io.s3_preds_info.ras_top     := RegNext(ram(io.s2_ftq_idx.value).ras_top)
    io.s3_preds_info.pc_debug    := pcs(s3_ftq_idx.value)
    io.s3_preds_info.preds_info  := s3_mem_preds_info
    // target pc 还需要一个额外的 bypass，因为要获取 target 是在 ftq_idx + 1，
    // 它可能恰好在 s2 周期才写入
    // val target_bypass = io.s2_ftq_idx + 1.U === f3_pred_enq_ptr && do_f3_preds_enq
    io.s3_preds_info.check_target := s3_mem_pred_target
    io.s3_preds_info.bpd_target   := s3_mem_pred_target
  }

  // predecode 重定向
  when (io.predecode_redirect.valid) {
    f3_pred_enq_ptr := io.predecode_redirect.bits + 1.U
    last_f3_enq_target := io.merged_f3_target
  }

  val do_predecode_enq = io.predecode_enq.valid


  // This register lets us initialize the ghist to 0
  val prev_ghist = RegInit((0.U).asTypeOf(new GlobalHistory))
  val prev_entry = RegInit((0.U).asTypeOf(new FTQBundle))
  val prev_pc    = RegInit(0.U(vaddrBitsExtended.W))
  when (do_predecode_enq) {

    val new_entry = Wire(new FTQBundle)

    new_entry.cfi_idx   := io.predecode_enq.bits.cfi_idx
    // Initially, if we see a CFI, it is assumed to be taken.
    // Branch resolutions may change this
    new_entry.cfi_taken     := io.predecode_enq.bits.cfi_idx.valid
    new_entry.cfi_mispredicted := false.B
    new_entry.cfi_type      := io.predecode_enq.bits.cfi_type
    new_entry.cfi_is_call   := io.predecode_enq.bits.cfi_is_call
    new_entry.cfi_is_ret    := io.predecode_enq.bits.cfi_is_ret
    new_entry.cfi_npc_plus4 := io.predecode_enq.bits.cfi_npc_plus4
    new_entry.ras_top       := io.predecode_enq.bits.ras_top
    new_entry.ras_idx       := io.predecode_enq.bits.ghist.ras_idx
    new_entry.br_mask       := io.predecode_enq.bits.br_mask
    new_entry.btb_mispredicts := io.predecode_enq.bits.btb_mispredicts
    new_entry.ft_tsrc       := io.predecode_enq.bits.tsrc
    new_entry.has_exp       := false.B
    new_entry.start_bank    := bank(io.predecode_enq.bits.pc)

    val new_ghist = io.predecode_enq.bits.ghist

    ram(predecode_enq_ptr.value) := new_entry

    prev_pc    := io.predecode_enq.bits.pc
    prev_entry := new_entry
    prev_ghist := new_ghist

    icache_stall_cycles.write(predecode_enq_ptr.value, io.predecode_enq.bits.ic_miss_stall_cycles)

    next_predecode_enq_ptr_debug := next_predecode_enq_ptr_debug + 1.U
  }

  io.bpdupdate.valid := false.B
  io.bpdupdate.bits  := DontCare

  when (io.deq.valid) {
    val new_deq_ptr1 = FTQPtr(deq_ptr.flag, io.deq.bits)
    val new_deq_ptr2 = FTQPtr.inverse(new_deq_ptr1)
    // 因为 uop 中只保留了 ftq.value，没有 ftq.flag，我们需要从
    // rob 返回的 ftq.value 中推断 flag 的值。如果 io.deq.bits
    // 的值与 deq_ptr.value 相同，说明新的 commit 位于同一个 fetch
    // packet，我们保持 deq_ptr 不变。否则，由于 deq_ptr 应当递增，
    // 我们选择使得 deq_ptr 不会减小的那个
    deq_ptr := Mux(deq_ptr <= new_deq_ptr1, new_deq_ptr1, new_deq_ptr2)
  }

  // We can update the branch predictors when we know the target of the
  // CFI in this fetch bundle

  val ras_update = WireInit(false.B)
  val ras_update_pc = WireInit(0.U(vaddrBitsExtended.W))
  val ras_update_idx = WireInit(0.U(log2Ceil(nRasEntries).W))
  io.ras_update     := RegNext(ras_update)
  io.ras_update_pc  := RegNext(ras_update_pc)
  io.ras_update_idx := RegNext(ras_update_idx)

  val bpd_update_mispredict = RegInit(false.B)
  val bpd_update_repair = RegInit(false.B)
  val bpd_repair_idx = Reg(UInt(log2Ceil(ftqSz).W))
  val bpd_end_idx = Reg(UInt(log2Ceil(ftqSz).W))
  val bpd_repair_pc = Reg(UInt(vaddrBitsExtended.W))

  val useLoop = p(BoomLoopKey)
  val bpd_idx = Wire(UInt(log2Ceil(ftqSz).W))
  if (useLoop) {
    bpd_idx := Mux(bpd_update_repair || bpd_update_mispredict, bpd_repair_idx, bpd_commit_ptr.value)
  } else {
    bpd_idx := bpd_commit_ptr.value
  }
  val bpd_entry = RegNext(ram(bpd_idx))
  val bpd_ghist = ghist(0).read(bpd_idx, true.B)

  val bpd_meta  = meta.read(bpd_idx, true.B) // TODO fix these SRAMs
  val bpd_pc    = RegNext(pcs(bpd_idx))

  val com_ic_stall_cycles = icache_stall_cycles.read(bpd_idx, true.B)
  io.commit_ic_stall_cycles := com_ic_stall_cycles
  // TODO: 不怎么写不知道为啥 verilator 那边模拟的时候显示
  // 当 bpd_ptr == 31 时做 commit update 会有问题，波形图
  // 显示输出的 bpd_target 为 0，不知道是不是 verilator 的 bug？
  val next_bpd_idx = Wire(UInt(log2Ceil(ftqSz).W))
  next_bpd_idx := WrapInc(bpd_idx, num_entries)
  dontTouch(next_bpd_idx)

  val readout_target = Wire(UInt(vaddrBitsExtended.W))
  readout_target := pcs(next_bpd_idx)
  dontTouch(readout_target)

  val bpd_target = RegNext(readout_target)

  when (io.redirect.valid) {
    bpd_update_mispredict := false.B
    bpd_update_repair     := false.B
  } .elsewhen (RegNext(io.brupdate.b2.mispredict)) {
    bpd_update_mispredict := true.B
    bpd_repair_idx        := RegNext(io.brupdate.b2.uop.ftq_idx)
    bpd_end_idx           := RegNext(f3_pred_enq_ptr.value)
  } .elsewhen (bpd_update_mispredict) {
    bpd_update_mispredict := false.B
    bpd_update_repair     := true.B
    bpd_repair_idx        := WrapInc(bpd_repair_idx, num_entries)
  } .elsewhen (bpd_update_repair && RegNext(bpd_update_mispredict)) {
    bpd_repair_pc         := bpd_pc
    bpd_repair_idx        := WrapInc(bpd_repair_idx, num_entries)
  } .elsewhen (bpd_update_repair) {
    bpd_repair_idx        := WrapInc(bpd_repair_idx, num_entries)
    when (WrapInc(bpd_repair_idx, num_entries) === bpd_end_idx ||
      bpd_pc === bpd_repair_pc)  {
      bpd_update_repair := false.B
    }

  }

  // 这里的判断中将原来的
  // bpd_ptr =/= deq_ptr && enq_ptr =/= WrapInc(bpd_ptr, num_entries)
  // 简化为了 bpd_ptr =/= deq_ptr，因为 deq_ptr < enq_ptr 在第一次 commit
  // update 后总是成立，而 bpd_ptr <= deq_ptr
  val do_commit_update = Wire(Bool())
  val do_mispredict_update = Wire(Bool())
  val do_repair_update = Wire(Bool())
  if (useLoop) {
    do_commit_update     := (bpd_commit_ptr =/= deq_ptr &&
                            !io.redirect.valid && !RegNext(io.redirect.valid)) &&
                            !bpd_update_mispredict && !bpd_update_repair
    do_mispredict_update := bpd_update_mispredict
    do_repair_update     := bpd_update_repair
  } else {
    do_commit_update     := (bpd_commit_ptr =/= deq_ptr &&
                            !io.redirect.valid && !RegNext(io.redirect.valid))
    do_mispredict_update := false.B
    do_repair_update     := false.B
  }

  val done_commit_update_debug = RegInit(false.B)

  io.ft_tsrc.valid := false.B
  io.ft_tsrc.bits  := DontCare
  when (RegNext(do_commit_update || do_repair_update || do_mispredict_update)) {
    val cfi_idx = bpd_entry.cfi_idx.bits
    val valid_repair = bpd_pc =/= bpd_repair_pc

    io.bpdupdate.valid := (bpd_entry.cfi_idx.valid || bpd_entry.br_mask =/= 0.U) &&
                          !(RegNext(do_repair_update) && !valid_repair)
    io.bpdupdate.bits.is_mispredict_update := RegNext(do_mispredict_update)
    io.bpdupdate.bits.is_repair_update     := RegNext(do_repair_update)
    io.bpdupdate.bits.pc      := bpd_pc
    io.bpdupdate.bits.btb_mispredicts := bpd_entry.btb_mispredicts
    io.bpdupdate.bits.br_mask :=  bpd_entry.br_mask
    io.bpdupdate.bits.cfi_idx := bpd_entry.cfi_idx
    io.bpdupdate.bits.cfi_mispredicted := bpd_entry.cfi_mispredicted
    io.bpdupdate.bits.cfi_taken  := bpd_entry.cfi_taken
    io.bpdupdate.bits.target     := bpd_target
    io.bpdupdate.bits.cfi_is_br  := bpd_entry.br_mask(cfi_idx)
    io.bpdupdate.bits.cfi_is_jal := bpd_entry.cfi_type === CFI_JAL || bpd_entry.cfi_type === CFI_JALR
    io.bpdupdate.bits.ghist      := bpd_ghist
    io.bpdupdate.bits.lhist      := DontCare
    io.bpdupdate.bits.meta       := bpd_meta
    io.bpdupdate.bits.cfi_is_call  := bpd_entry.cfi_is_call
    io.bpdupdate.bits.cfi_is_ret   := bpd_entry.cfi_is_ret
    io.bpdupdate.bits.cfi_npc_plus4 := bpd_entry.cfi_npc_plus4

    io.ft_tsrc.valid := true.B
    io.ft_tsrc.bits  := bpd_entry.ft_tsrc

    done_commit_update_debug := RegNext(do_commit_update) || done_commit_update_debug
  }

  when (do_commit_update) {
    bpd_commit_ptr := bpd_commit_ptr + 1.U
  }

  val last_commit_cfi_type = RegInit(IC_MISS_SEQ)
  io.last_commit_cfi_type := last_commit_cfi_type
  io.update_miss_counter := false.B
  when (RegNext(do_commit_update)) {
    io.update_miss_counter := true.B
    // 如果发生了异常
    when (bpd_entry.has_exp) {
      last_commit_cfi_type := IC_MISS_EXCEPTION
    // 如果没有控制流指令或者控制流指令最终没有 taken
    } .elsewhen(!bpd_entry.cfi_idx.valid || !bpd_entry.cfi_taken) {
      last_commit_cfi_type := IC_MISS_SEQ
    } .elsewhen(bpd_entry.cfi_type === CFI_JAL) {
      last_commit_cfi_type := IC_MISS_JAL
    } .elsewhen(bpd_entry.cfi_type === CFI_BR) {
      last_commit_cfi_type := IC_MISS_COND
    } .elsewhen(bpd_entry.cfi_is_ret) {
      last_commit_cfi_type := IC_MISS_RET
    } .otherwise {
      last_commit_cfi_type := IC_MISS_JALR
    }
  }

  // 因为 ghist 用的 sync read mem，文档里说不能在同一周期读和写相同地址
  // 因此这里就保守一些，在 ftq 满的时候就拉低 ready 信号，即使这周期的
  // do_commit_update 信号为 true

  val redirect_idx = io.redirect.bits
  val redirect_entry = ram(redirect_idx)
  val redirect_new_entry = WireInit(redirect_entry)

  // Debug usage
  val bpd_idx_debug = io.redirect.bits
  val bpd_ghist_debug = ghist(0).read(bpd_idx_debug, true.B)
  val bpd_pc_debug = RegNext(pcs(bpd_idx_debug))

  io.redirect_ftq_idx := DontCare
  when (io.redirect.valid) {
    // 类似于 deq_ptr 的处理。因为新的 enq_ptr 只可能减小或者不变
    val new_enq_ptr1 = FTQPtr(next_predecode_enq_ptr_debug.flag, io.redirect.bits)
    val new_enq_ptr2 = FTQPtr.inverse(new_enq_ptr1)
    val proper_enq_ptr = Mux(next_predecode_enq_ptr_debug > new_enq_ptr1, new_enq_ptr1, new_enq_ptr2)
    next_predecode_enq_ptr_debug    := proper_enq_ptr + 1.U
    io.redirect_ftq_idx := proper_enq_ptr

    val f3_pred_enq_ptr1 = FTQPtr(f3_pred_enq_ptr.flag, io.redirect.bits)
    val f3_pred_enq_ptr2 = FTQPtr.inverse(f3_pred_enq_ptr1)
    val proper_f3_pred_enq_ptr = Mux(f3_pred_enq_ptr > f3_pred_enq_ptr1, f3_pred_enq_ptr1, f3_pred_enq_ptr2)
    f3_pred_enq_ptr := proper_f3_pred_enq_ptr + 1.U
    last_f3_enq_target := io.redirect_target

    when (io.rob_flush) {
      val new_cfi_idx = (io.rob_flush_pc_lob ^
      Mux(redirect_entry.start_bank === 1.U, 1.U << log2Ceil(bankBytes), 0.U))(log2Ceil(fetchWidth), 1)
      // 例如 fetch packet 的四条指令是 br, nop, load, br
      // 我们预测第四条 br 为 taken，但 load 指令发送了异常
      // 因此需要在 rob flush 时，修正 cfi_idx 和 br_mask
      redirect_new_entry.br_mask := MaskLower(UIntToOH(new_cfi_idx)) & redirect_entry.br_mask
      redirect_new_entry.cfi_idx.valid    := false.B
      redirect_new_entry.has_exp          := true.B
    } .elsewhen (io.brupdate.b2.mispredict) {
    val new_cfi_idx = (io.brupdate.b2.uop.pc_lob ^
      Mux(redirect_entry.start_bank === 1.U, 1.U << log2Ceil(bankBytes), 0.U))(log2Ceil(fetchWidth), 1)
      redirect_new_entry.cfi_idx.valid    := true.B
      redirect_new_entry.cfi_idx.bits     := new_cfi_idx
      redirect_new_entry.cfi_mispredicted := true.B
      redirect_new_entry.cfi_taken        := io.brupdate.b2.taken
      redirect_new_entry.cfi_is_call      := redirect_entry.cfi_is_call && redirect_entry.cfi_idx.bits === new_cfi_idx
      redirect_new_entry.cfi_is_ret       := redirect_entry.cfi_is_ret  && redirect_entry.cfi_idx.bits === new_cfi_idx
      // BOOM 原本的逻辑没有修改 cfi_type 和 br_mask
      // 但我觉得是应该加上的
      redirect_new_entry.cfi_type         := io.brupdate.b2.cfi_type
      redirect_new_entry.br_mask          := MaskLower(UIntToOH(new_cfi_idx)) & redirect_entry.br_mask
    }

    ras_update     := true.B
    ras_update_pc  := redirect_entry.ras_top
    ras_update_idx := redirect_entry.ras_idx

  } .elsewhen (RegNext(io.redirect.valid)) {
    prev_entry := RegNext(redirect_new_entry)
    prev_ghist := bpd_ghist_debug
    prev_pc    := bpd_pc_debug

    ram(RegNext(io.redirect.bits)) := RegNext(redirect_new_entry)
  }

  //-------------------------------------------------------------
  // **** Core Read PCs ****
  //-------------------------------------------------------------

  for (i <- 0 until 2) {
    val idx = io.get_ftq_pc(i).ftq_idx
    val next_idx = WrapInc(idx, num_entries)
    // TODO: remove this
    val next_is_enq = (next_idx === f3_pred_enq_ptr.value) && io.predecode_enq.fire
    val next_pc = Mux(next_is_enq, io.predecode_enq.bits.pc, pcs(next_idx))
    val get_entry = ram(idx)
    val next_entry = ram(next_idx)
    io.get_ftq_pc(i).entry     := RegNext(get_entry)
    if (i == 1)
      io.get_ftq_pc(i).ghist   := ghist(1).read(idx, true.B)
    else
      io.get_ftq_pc(i).ghist   := DontCare
    io.get_ftq_pc(i).pc        := RegNext(pcs(idx))
    io.get_ftq_pc(i).next_pc   := RegNext(next_pc)
    io.get_ftq_pc(i).next_val  := RegNext(next_idx =/= f3_pred_enq_ptr.value || next_is_enq)
    
    val com_pc_ptr = Wire(UInt(log2Ceil(ftqSz).W))
    com_pc_ptr := Mux(io.deq.valid, io.deq.bits, deq_ptr.value)
    io.get_ftq_pc(i).com_pc    := RegNext(pcs(com_pc_ptr))
  }

  for (w <- 0 until coreWidth) {
    io.debug_fetch_pc(w) := RegNext(pcs(io.debug_ftq_idx(w)))
  }

  // Printf
  if (IN_SIMULATION) {
    // Add a free-running cycle counter for simulation
    val (cycleCount, _) = Counter(true.B, Int.MaxValue)
    val prev_update_printf = PlusArg("prev-ras-update", 0, "Print FTQ prev entry update", 1)
    when (do_predecode_enq && prev_update_printf(0)) {
       val (ghist_by_prev, _) = prev_ghist.update(
        prev_entry.br_mask,
        prev_entry.cfi_taken,
        prev_entry.br_mask(prev_entry.cfi_idx.bits),
        prev_entry.cfi_idx.bits,
        prev_entry.cfi_idx.valid,
        prev_pc,
        prev_entry.cfi_is_call,
        prev_entry.cfi_is_ret
      )
      when (io.predecode_enq.bits.ghist.current_saw_branch_not_taken) {
        ghist_by_prev := io.predecode_enq.bits.ghist
      }
      when (ghist_by_prev.ras_idx =/= prev_ghist.ras_idx || 
            io.predecode_enq.bits.ghist.ras_idx =/= prev_ghist.ras_idx) {
        printf(p"[${cycleCount} FTQ] prev_ras_idx=${Hexadecimal(prev_ghist.ras_idx)}, " +
          p"prev_cfi_call=${prev_entry.cfi_is_call && prev_entry.cfi_idx.valid}, " +
          p"prev_cfi_ret=${prev_entry.cfi_is_ret && prev_entry.cfi_idx.valid}, " +
          p"prev_pc=${Hexadecimal(prev_pc)}, " +
          p"now_ras_idx=${Hexadecimal(ghist_by_prev.ras_idx)}, " +
          p"real_ras_idx=${Hexadecimal(io.predecode_enq.bits.ghist.ras_idx)}\n")
      }
    }
    val enq_ghist_printf = PlusArg("enq-ghist-update", 0, "Print FTQ enq ghist update", 1)
    when (do_predecode_enq && enq_ghist_printf(0)) {
      printf(p"[${cycleCount} FTQ] enq_ptr=${predecode_enq_ptr.value}, enq_pc=${Hexadecimal(io.predecode_enq.bits.pc)}, " +
        p"enq_cfi_idx=${io.predecode_enq.bits.cfi_idx}, " +
        p"enq_br_mask=${Binary(io.predecode_enq.bits.br_mask)}, " +
        p"enq_ghist=${Binary(io.predecode_enq.bits.ghist.histories(0))}\n")
    }
    val bpd_ahead_printf = PlusArg("bpd-ahead-printf", 0, "Print FTQ bpd ahead of predecode", 1)
    when (do_f3_preds_enq && bpd_ahead_printf(0)) {
      val distance = distanceBetween(f3_pred_enq_ptr, next_predecode_enq_ptr_debug)
      printf(p"[${cycleCount} FTQ] f3_pred_enq_ptr=${f3_pred_enq_ptr}, " +
        p"predecode_enq_ptr=${next_predecode_enq_ptr_debug}, " +
        p"distance=${distance}\n")
    }
  }

  // Assertion
  when (do_predecode_enq) {
    assert (full === false.B, "We should have allocated FTQ space in s0")

    // 前端在计算 f3_br_mask 时已经考虑了 fetch bundle 内部的 mask 了
    assert ((io.predecode_enq.bits.br_mask & io.predecode_enq.bits.mask) === io.predecode_enq.bits.br_mask,
      "FTQ: br_mask has bits set outside of valid fetch bundle")
    
    when (!io.predecode_enq.bits.ghist.current_saw_branch_not_taken) {
      val (ghist_by_prev, _) = prev_ghist.update(
        prev_entry.br_mask,
        prev_entry.cfi_taken,
        prev_entry.br_mask(prev_entry.cfi_idx.bits),
        prev_entry.cfi_idx.bits,
        prev_entry.cfi_idx.valid,
        prev_pc,
        prev_entry.cfi_is_call,
        prev_entry.cfi_is_ret
      )
      // 按我的理解这俩应该是相等的
      assert (io.predecode_enq.bits.ghist === ghist_by_prev,
        "FTQ: ghist mismatch on enqueue. ")
      // 暂时关闭 RAS 相关的 assertion
      // assert (io.predecode_enq.bits.ghist.ras_idx === ghist_by_prev.ras_idx,
      //   p"FTQ: ras_idx mismatch on enqueue. enq: ${io.predecode_enq.bits.ghist.ras_idx}, prev: ${ghist_by_prev.ras_idx}")
    }
  }
  
  assert (deq_ptr < next_predecode_enq_ptr_debug || !done_commit_update_debug,
          """After commit update, deq_ptr should always point to 
          |a valid fetch packet, and deq_ptr points to next enqueue position"""
  )

  assert (bpd_commit_ptr <= deq_ptr, "bpd_ptr should always behind deq_ptr")
  when (io.deq.valid) {
    // 因为 rob 每周期至多提交 coreWidth 条指令，而一条指令最多占据两
    // 个 fetch packet。因此每次 commit, deq_ptr 至多前进 2 * coreWidth
    // 个位置
    val possible_new_ptrs = Seq.tabulate(2 * coreWidth + 1) { i => (deq_ptr + i.U).value === io.deq.bits }
    assert (possible_new_ptrs.reduce(_ || _),
    "deq_ptr should be incremented by at most 2 * coreWidth?")
  }

  when (do_commit_update) {
    // 这保证了在同一周期 ghist 的 commit 读和 enq 写不会是同一地址
    // 没有检测 mispredict 和 repair update，因为我们暂时不用 loop predictor
    // 没有检测 redirect ghist read，因为 BOOM 里没有一个 valid 信号
    // 来指示 io.get_ftq_pc(i).ftq_idx 是否有效
    assert (bpd_commit_ptr < next_predecode_enq_ptr_debug, "bpd_ptr should never commit a invalid ftq entry")
  }
  when (io.bpdupdate.valid) {
    assert (io.bpdupdate.bits.target =/= 0.U,
      "FTQ: branch target cannot be 0")
    
    when (bpd_entry.cfi_idx.valid) {
      assert ((MaskLower(UIntToOH(bpd_entry.cfi_idx.bits)) & bpd_entry.br_mask) === bpd_entry.br_mask,
        "FTQ: bpd_update br_mask has bits set outside of valid fetch bundle")
    }
  }
  when (io.predecode_enq.fire) {
    assert (next_predecode_enq_ptr_debug === predecode_enq_ptr)
  }
  when (io.f3_preds_enq.valid) {
    assert (f3_pred_enq_ptr === io.f3_ftq_idx_debug,
      "FTQ: f3_pred_enq_ptr should always equal f3 preds ftq idx")
    when (io.predecode_enq.valid) {
      assert (f3_pred_enq_ptr > predecode_enq_ptr,
        "FTQ: f3_pred enqueue should always be ahead of predecode enqqueue" )
      assert (isFull(f3_pred_enq_ptr, predecode_enq_ptr) === false.B,
        "FTQ: f3_pred enqueue should never pass predecode enqueue" )
    }
  }
  assert (f3_pred_enq_ptr >= next_predecode_enq_ptr_debug ||
          isFull(f3_pred_enq_ptr, next_predecode_enq_ptr_debug),
          "FTQ: f3_pred_enq_ptr should always be ahead of predecode_enq_ptr")
  
  if (IN_SIMULATION) {
    // Track which FTQ entries are valid (for assertions only).
    val ftq_entry_valid = RegInit(VecInit(Seq.fill(num_entries)(false.B)))

    // 1) predecode_enq: allocated entry must be invalid, then becomes valid.
    when (do_predecode_enq) {
      assert (!ftq_entry_valid(predecode_enq_ptr.value),
        "FTQ: predecode enqueue should allocate an invalid entry")
      ftq_entry_valid(predecode_enq_ptr.value) := true.B
    }

    // 2) redirect: redirect entry must be valid; clear (redirect.bits, last_enq) on the ring.
    when (io.redirect.valid) {
      val redirect_idx = io.redirect.bits
      val last_enq_idx = next_predecode_enq_ptr_debug.value

      assert (ftq_entry_valid(redirect_idx),
        "FTQ: redirect should point to a valid FTQ entry")

      for (i <- 0 until num_entries) {
        val idx = i.U
        val in_range = Mux(last_enq_idx > redirect_idx,
          idx > redirect_idx && idx < last_enq_idx,
          idx > redirect_idx || idx < last_enq_idx)
        when (in_range) {
          assert (ftq_entry_valid(i), "FTQ: the entry will be flushed should be valid")
          ftq_entry_valid(i) := false.B
        }
        when (idx === last_enq_idx && do_predecode_enq) {
          ftq_entry_valid(i) := false.B
        }
      }
    }

    // 3) s3_preds_info: the FTQ entry being read should not have been allocated yet.
    when (do_f3_preds_enq) {
      assert (!ftq_entry_valid(f3_pred_enq_ptr.value),
        "FTQ: s3 preds info should read an unallocated FTQ entry")
    }

    // 4) commit: entry being committed must be valid, then becomes invalid.
    when (do_commit_update) {
      val commit_idx = bpd_commit_ptr.value
      assert (ftq_entry_valid(commit_idx),
        "FTQ: commit update should operate on a valid FTQ entry")
      val commit_target_idx = WrapInc(commit_idx, num_entries)
      assert (ftq_entry_valid(commit_target_idx),
        "FTQ: next entry for commit update should be valid to ensure the target is valid")
      ftq_entry_valid(commit_idx) := false.B
    }
  }

  // TODO: 引入 valid 位检查一下 get_pc 相关的 ftq idx 是否正确
}

//******************************************************************************
// Copyright (c) 2017 - 2019, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Frontend
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom.v3.ifu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config._
import freechips.rocketchip.subsystem._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.rocket._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.tile._
import freechips.rocketchip.util._
import freechips.rocketchip.util.property._

import boom.v3.common._
import boom.v3.exu.{CommitExceptionSignals, BranchDecode, BrUpdateInfo, BranchDecodeSignals}
import boom.v3.util._


class FrontendResp(implicit p: Parameters) extends BoomBundle()(p) {
  val ftq_idx = new FTQPtr
  val pc = UInt(vaddrBitsExtended.W)  // ID stage PC
  val data = UInt((fetchWidth * coreInstBits).W)
  val mask = UInt(fetchWidth.W)
  val xcpt = new FrontendExceptions
  val tsrc = UInt(BSRC_SZ.W)

  // ICache miss-stall cycles carried with this frontend response.
  val ic_miss_stall_cycles = UInt(12.W)
}

class GlobalHistory(implicit p: Parameters) extends BoomBundle()(p)
  with HasBoomFrontendParameters
{
  // For the dual banked case, each bank ignores the contribution of the
  // last bank to the history. Thus we have to track the most recent update to the
  // history in that case
  val old_history = UInt(globalHistoryLength.W)

  val current_saw_branch_not_taken = Bool()

  val new_saw_branch_not_taken = Bool()
  val new_saw_branch_taken     = Bool()

  val ras_idx = UInt(log2Ceil(nRasEntries).W)

  def histories(bank: Int) = {
    if (nBanks == 1) {
      old_history
    } else {
      require(nBanks == 2)
      if (bank == 0) {
        old_history
      } else {
        Mux(new_saw_branch_taken                            , old_history << 1 | 1.U,
        Mux(new_saw_branch_not_taken                        , old_history << 1,
                                                              old_history))
      }
    }
  }

  def ===(other: GlobalHistory): Bool = {
    ((old_history === other.old_history) &&
     (new_saw_branch_not_taken === other.new_saw_branch_not_taken) &&
     (new_saw_branch_taken === other.new_saw_branch_taken)
    )
  }
  def =/=(other: GlobalHistory): Bool = !(this === other)

  def update(branches: UInt, cfi_taken: Bool, cfi_is_br: Bool, cfi_idx: UInt,
    cfi_valid: Bool, addr: UInt,
    cfi_is_call: Bool, cfi_is_ret: Bool): (GlobalHistory, UInt) = {
    val cfi_idx_fixed = cfi_idx(log2Ceil(fetchWidth)-1,0)
    val cfi_idx_oh = UIntToOH(cfi_idx_fixed)
    val new_history = Wire(new GlobalHistory)

    val not_taken_branches = branches & Mux(cfi_valid,
                                            MaskLower(cfi_idx_oh) & ~Mux(cfi_is_br && cfi_taken, cfi_idx_oh, 0.U(fetchWidth.W)),
                                            ~(0.U(fetchWidth.W)))
    val ghr_update_type = WireDefault(NO_SHIFT)

    if (nBanks == 1) {
      // In the single bank case every bank sees the history including the previous bank
      new_history := DontCare
      new_history.current_saw_branch_not_taken := false.B
      val saw_not_taken_branch = not_taken_branches =/= 0.U || current_saw_branch_not_taken
      new_history.old_history := Mux(cfi_is_br && cfi_taken && cfi_valid   , histories(0) << 1 | 1.U,
                                 Mux(saw_not_taken_branch                  , histories(0) << 1,
                                                                             histories(0)))
      ghr_update_type := Mux(cfi_is_br && cfi_taken && cfi_valid, SHIFT_ONE,
                           Mux(saw_not_taken_branch, SHIFT_ZERO, NO_SHIFT))
    } else {
      // In the two bank case every bank ignore the history added by the previous bank
      val base = histories(1)
      val cfi_in_bank_0 = cfi_valid && cfi_taken && cfi_idx_fixed < bankWidth.U
      val ignore_second_bank = cfi_in_bank_0 || mayNotBeDualBanked(addr)

      val first_bank_saw_not_taken = not_taken_branches(bankWidth-1,0) =/= 0.U || current_saw_branch_not_taken
      new_history.current_saw_branch_not_taken := false.B
      when (ignore_second_bank) {
        new_history.old_history := histories(1)
        new_history.new_saw_branch_not_taken := first_bank_saw_not_taken
        new_history.new_saw_branch_taken     := cfi_is_br && cfi_in_bank_0
      } .otherwise {
        new_history.old_history := Mux(cfi_is_br && cfi_in_bank_0                             , histories(1) << 1 | 1.U,
                                   Mux(first_bank_saw_not_taken                               , histories(1) << 1,
                                                                                                histories(1)))

        new_history.new_saw_branch_not_taken := not_taken_branches(fetchWidth-1,bankWidth) =/= 0.U
        new_history.new_saw_branch_taken     := cfi_valid && cfi_taken && cfi_is_br && !cfi_in_bank_0

      }
    }
    new_history.ras_idx := Mux(cfi_valid && cfi_is_call, WrapInc(ras_idx, nRasEntries),
                           Mux(cfi_valid && cfi_is_ret , WrapDec(ras_idx, nRasEntries), ras_idx))

    (new_history, ghr_update_type)
  }

}

/**
 * Parameters to manage a L1 Banked ICache
 */
trait HasBoomFrontendParameters extends HasL1ICacheParameters
{
  // How many banks does the ICache use?
  val nBanks = if (cacheParams.fetchBytes <= 8) 1 else 2
  // How many bytes wide is a bank?
  val bankBytes = fetchBytes/nBanks

  val bankWidth = fetchWidth/nBanks

  require(nBanks == 1 || nBanks == 2)



  // How many "chunks"/interleavings make up a cache line?
  val numChunks = cacheParams.blockBytes / bankBytes

  // Which bank is the address pointing to?
  def bank(addr: UInt) = if (nBanks == 2) addr(log2Ceil(bankBytes)) else 0.U
  def isLastBankInBlock(addr: UInt) = {
    (nBanks == 2).B && addr(blockOffBits-1, log2Ceil(bankBytes)) === (numChunks-1).U
  }
  def mayNotBeDualBanked(addr: UInt) = {
    require(nBanks == 2)
    isLastBankInBlock(addr)
  }

  def blockAlign(addr: UInt) = ~(~addr | (cacheParams.blockBytes-1).U)
  def bankAlign(addr: UInt) = ~(~addr | (bankBytes-1).U)

  def fetchIdx(addr: UInt) = addr >> log2Ceil(fetchBytes)

  def nextBank(addr: UInt) = bankAlign(addr) + bankBytes.U
  def nextFetch(addr: UInt) = {
    if (nBanks == 1) {
      bankAlign(addr) + bankBytes.U
    } else {
      require(nBanks == 2)
      bankAlign(addr) + Mux(mayNotBeDualBanked(addr), bankBytes.U, fetchBytes.U)
    }
  }

  def fetchMask(addr: UInt) = {
    val idx = addr.extract(log2Ceil(fetchWidth)+log2Ceil(coreInstBytes)-1, log2Ceil(coreInstBytes))
    if (nBanks == 1) {
      ((1 << fetchWidth)-1).U << idx
    } else {
      val shamt = idx.extract(log2Ceil(fetchWidth)-2, 0)
      val end_mask = Mux(mayNotBeDualBanked(addr), Fill(fetchWidth/2, 1.U), Fill(fetchWidth, 1.U))
      ((1 << fetchWidth)-1).U << shamt & end_mask
    }
  }

  def bankMask(addr: UInt) = {
    val idx = addr.extract(log2Ceil(fetchWidth)+log2Ceil(coreInstBytes)-1, log2Ceil(coreInstBytes))
    if (nBanks == 1) {
      1.U(1.W)
    } else {
      Mux(mayNotBeDualBanked(addr), 1.U(2.W), 3.U(2.W))
    }
  }
}



/**
 * Bundle passed into the FetchBuffer and used to combine multiple
 * relevant signals together.
 */
class FetchBundle(implicit p: Parameters) extends BoomBundle
  with HasBoomFrontendParameters
{
  val pc            = UInt(vaddrBitsExtended.W)
  val next_pc       = UInt(vaddrBitsExtended.W)
  val edge_inst     = Vec(nBanks, Bool()) // True if 1st instruction in this bundle is pc - 2
  val insts         = Vec(fetchWidth, Bits(32.W))
  val exp_insts     = Vec(fetchWidth, Bits(32.W))

  // Information for sfb folding
  // NOTE: This IS NOT equivalent to uop.pc_lob, that gets calculated in the FB
  val sfbs                 = Vec(fetchWidth, Bool())
  val sfb_masks            = Vec(fetchWidth, UInt((2*fetchWidth).W))
  val sfb_dests            = Vec(fetchWidth, UInt((1+log2Ceil(fetchBytes)).W))
  val shadowable_mask      = Vec(fetchWidth, Bool())
  val shadowed_mask        = Vec(fetchWidth, Bool())

  val cfi_idx       = Valid(UInt(log2Ceil(fetchWidth).W))
  val cfi_type      = UInt(CFI_SZ.W)
  val cfi_is_call   = Bool()
  val cfi_is_ret    = Bool()
  val cfi_npc_plus4 = Bool()

  val ras_top       = UInt(vaddrBitsExtended.W)

  val ftq_idx       = new FTQPtr
  val mask          = UInt(fetchWidth.W) // mark which words are valid instructions

  val br_mask       = UInt(fetchWidth.W)

  val ghist         = new GlobalHistory

  val xcpt_pf_if    = Bool() // I-TLB miss (instruction fetch fault).
  val xcpt_ae_if    = Bool() // Access exception.

  val bp_debug_if_oh= Vec(fetchWidth, Bool())
  val bp_xcpt_if_oh = Vec(fetchWidth, Bool())

  val end_half      = Valid(UInt(16.W))


  val bpd_meta      = Vec(nBanks, UInt())

  // btb 将非分支指令或者非指令预测为了分支指令
  val btb_mispredicts = UInt(fetchWidth.W)

  // Source of the prediction from this bundle
  val fsrc    = UInt(BSRC_SZ.W)
  // Source of the prediction to this bundle
  val tsrc    = UInt(BSRC_SZ.W)

  // ICache miss-stall cycles carried with this fetch packet.
  val ic_miss_stall_cycles = UInt(12.W)
}



/**
 * IO for the BOOM Frontend to/from the CPU
 */
class BoomFrontendIO(implicit p: Parameters) extends BoomBundle
{
  // Give the backend a packet of instructions.
  val fetchpacket       = Flipped(new DecoupledIO(new FetchBufferResp))
  val dec_fire          = Output(Vec(coreWidth, Bool()))
  val ft_tsrc           = Flipped(Valid(UInt(BSRC_SZ.W)))

  // 1 for xcpt/jalr/auipc/flush
  val get_pc            = Flipped(Vec(2, new GetPCFromFtqIO()))
  val mispredict_val    = Output(Bool()) // 表示 get_pc_1 是否有效
  val debug_ftq_idx     = Output(Vec(coreWidth, UInt(log2Ceil(ftqSz).W)))
  val debug_fetch_pc    = Input(Vec(coreWidth, UInt(vaddrBitsExtended.W)))

  // Breakpoint info
  val status            = Output(new MStatus)
  val bp                = Output(Vec(nBreakpoints, new BP))
  val mcontext          = Output(UInt(coreParams.mcontextWidth.W))
  val scontext          = Output(UInt(coreParams.scontextWidth.W))

  val sfence = Valid(new SFenceReq)

  val brupdate          = Output(new BrUpdateInfo)

  // Redirects change the PC
  val redirect_flush   = Output(Bool()) // Flush and hang the frontend?
  val redirect_val     = Output(Bool()) // Redirect the frontend?
  val redirect_pc      = Output(UInt()) // Where do we redirect to?
  val redirect_ftq_idx = Output(UInt()) // Which ftq entry should we reset to?
  val redirect_ghist   = Output(new GlobalHistory) // What are we setting as the global history?

  // Backend call mispredict: if the mispredicted CFI is a call, the core sends
  // the correct return address so the frontend can push it onto the RAS.
  val redirect_is_call  = Output(Bool())
  val redirect_call_addr = Output(UInt(vaddrBitsExtended.W))
  
  // 额外的 flush 信号，使得 FTQ 可以区分 rob flush 和 mispredict
  val rob_flush = Output(Bool()) // Flush coming from the ROB
  val rob_flush_pc_lob     = Output(UInt(log2Ceil(icBlockBytes).W))

  val commit = Valid(UInt(ftqSz.W))

  val flush_icache = Output(Bool())

  val perf = Input(new FrontendPerfEvents)

  ////Enable_PerfCounter_Support: for icache and information
  val itlb_valid_access = Input(Bool())
  val itlb_hit = Input(Bool())
  val icache_valid_access = Input(Bool())
  val icache_hit = Input(Bool())

  // Frontend s2 replay statistics
  val s2_replay_total   = Input(Bool())
  val s2_replay_itlb_miss = Input(Bool())
  val s2_replay_ic_miss   = Input(Bool())

  // ICache miss-stall cycles split by cause (frontend accumulates by packet, core does nibble mapping).
  val ic_miss_stall_seq       = Input(UInt(12.W))
  val ic_miss_stall_cond      = Input(UInt(12.W))
  val ic_miss_stall_jal       = Input(UInt(12.W))
  val ic_miss_stall_jalr      = Input(UInt(12.W))
  val ic_miss_stall_ret       = Input(UInt(12.W))
  val ic_miss_stall_exception = Input(UInt(12.W))

  // Fetch buffer enqueue monitor (for perf counters)
  val fb_enq_valid      = Input(Bool())
  val fb_enq_cnt        = Input(UInt(log2Ceil(fetchWidth+1).W))

  // BPD-ahead-of-IFU distance bucket (7 ranges)
  val bpd_ifu_dist_bucket = Input(UInt(3.W))

  // Prefetch refill distance bucket: valid when a prefetch MSHR completes
  // writeback and not cancelled by fencei, carrying the distance bucket
  // recorded at the time prefetch was issued
  val pf_refill_dist_bucket = Flipped(Valid(UInt(3.W)))

  // Prefetch accuracy: true when a prefetch-filled line is first accessed by IFU
  val pf_hit_success = Input(Bool())

  // Dynamic UFTQ-AUR prefetch distance samples for perf counters
  val dynamic_pf_dist_sample_count = Input(Bool())
  val dynamic_pf_dist_sum = Input(UInt(8.W))

  // Frontend s0 stall statistics
  //  - s0_ifu_real_not_valid: s0_ifu_real_valid is false
  //  - s0_ifu_ftq_backpress: s0_valid && ifu_to_ftq_not_ready
  val s0_ifu_real_not_valid = Input(Bool())
  val s0_ifu_ftq_backpress  = Input(Bool())

  // Predecode redirect pulse for perf counters
  val predecode_redirect_count = Input(Bool())

  // Frontend bubble statistics by clear source
  // Bubble count is based on distanceBetween(s0_ifu_ftq_idx_reg, s0_ftq_idx)
  //  - f2_clear_bubble/f3_clear_bubble/predecode_clear_bubble: low 4 bits
  //  - rob_flush_bubble/mispred_flush_bubble: full 6-bit bubble value (0-32)
  val f2_clear_bubble        = Input(UInt(4.W))
  val f3_clear_bubble        = Input(UInt(4.W))
  val predecode_clear_bubble = Input(UInt(4.W))
  val rob_flush_bubble       = Input(UInt((log2Ceil(ftqSz)+1).W))
  val mispred_flush_bubble   = Input(UInt((log2Ceil(ftqSz)+1).W))
}

/**
 * Top level Frontend class
 *
 * @param icacheParams parameters for the icache
 * @param hartid id for the hardware thread of the core
 */
class BoomFrontend(val icacheParams: ICacheParams, staticIdForMetadataUseOnly: Int)(implicit p: Parameters) extends LazyModule
{
  lazy val module = new BoomFrontendModule(this)
  val icache = LazyModule(new boom.v3.ifu.ICache(icacheParams, staticIdForMetadataUseOnly))
  val masterNode = icache.masterNode
  val resetVectorSinkNode = BundleBridgeSink[UInt](Some(() =>
    UInt(masterNode.edges.out.head.bundle.addressBits.W)))
}

/**
 * Bundle wrapping the IO for the Frontend as a whole
 *
 * @param outer top level Frontend class
 */
class BoomFrontendBundle(val outer: BoomFrontend) extends CoreBundle()(outer.p)
{
  val cpu = Flipped(new BoomFrontendIO())
  val ptw = new TLBPTWIO()
}

class ITLBLookup(implicit p: Parameters) extends BoomModule()(p)
{
  // Use CoreBundle the same way as RocketCore (new CoreBundle()(p) { ... })
  // so that the underlying ParameterizedBundle is properly initialized.
  val io = IO(new CoreBundle()(p) {
    val enq = Flipped(Decoupled(new TLBResp(log2Ceil(fetchBytes))))
    val enq_ftq_idx = Input(new FTQPtr)
    val deq = Decoupled(new TLBResp(log2Ceil(fetchBytes)))
    val flush = Input(Bool())
    val bpd_f3_flush = Input(Bool())
    val f3_ftq_idx = Input(new FTQPtr)

    val deq_ftq_idx_debug = Output(new FTQPtr)
    val predecode_redirect_debug = Input(Bool())
  })

  val last_entry = Reg(new TLBResp(log2Ceil(fetchBytes)))
  val last_ftq_idx = Reg(new FTQPtr)
  val last_valid = RegInit(false.B)

  val trans_queue = withReset(reset.asBool || io.flush) {
    Module(new Queue(new TLBResp(log2Ceil(fetchBytes)), ftqSz, flow = true))
  }

  val last_push_flush = io.bpd_f3_flush && (io.f3_ftq_idx < last_ftq_idx)
  val kill_last = last_push_flush || io.flush

  // Internal queue always accepts entries; we rely on ftqSz being large
  // enough and assert this condition.
  trans_queue.io.enq.valid := last_valid && !kill_last
  trans_queue.io.enq.bits  := last_entry

  // Dequeue side just mirrors the internal queue.
  trans_queue.io.deq.ready := io.deq.ready
  // 维持 flow 特性
  io.deq.valid             := trans_queue.io.deq.valid || io.enq.valid
  io.deq.bits              := trans_queue.io.deq.bits
  when (!trans_queue.io.deq.valid) {
    io.deq.bits := io.enq.bits
  }

  // Always ready to accept an enqueue; elements are first stored in
  // last_entry and committed into trans_queue in the next cycle.
  io.enq.ready := true.B

  last_entry := io.enq.bits
  last_ftq_idx := io.enq_ftq_idx
  // enq fire 的同时且没有直接 flow 出去, 该表项才会进入到 last_entry 中
  last_valid := io.enq.fire && (!io.deq.fire || trans_queue.io.deq.valid)

  assert(trans_queue.io.enq.ready, "ITLBLookup: internal queue should never be full")
  when (last_valid && io.bpd_f3_flush && !io.predecode_redirect_debug) {
    assert(io.f3_ftq_idx >= last_ftq_idx || io.f3_ftq_idx + 1.U === last_ftq_idx,
      "ITLBLookup: cannot flush more than one entry at a time")
  }

  io.deq_ftq_idx_debug := DontCare

  if (IN_SIMULATION) {
    val ftq_idx_queue = withReset(reset.asBool || io.flush) {
      Module(new Queue(new FTQPtr, ftqSz, flow = true))
    }
    ftq_idx_queue.io.enq.valid := last_valid && !kill_last
    ftq_idx_queue.io.enq.bits  := last_ftq_idx
    ftq_idx_queue.io.deq.ready := io.deq.ready
    io.deq_ftq_idx_debug := Mux(ftq_idx_queue.io.deq.valid, ftq_idx_queue.io.deq.bits, io.enq_ftq_idx)
    
    assert (trans_queue.io.count === ftq_idx_queue.io.count,
      "ITLBLookup: internal queue and ftq idx queue count should match")
  }
}

/**
 * Main Frontend module that connects the icache, TLB, fetch controller,
 * and branch prediction pipeline together.
 *
 * @param outer top level Frontend class
 */
class BoomFrontendModule(outer: BoomFrontend) extends LazyModuleImp(outer)
  with HasBoomCoreParameters
  with HasBoomFrontendParameters
  with HasCircularQueuePtrHelper
{
  val io = IO(new BoomFrontendBundle(outer))
  val io_reset_vector = outer.resetVectorSinkNode.bundle
  implicit val edge = outer.masterNode.edges.out(0)
  require(fetchWidth*coreInstBytes == outer.icacheParams.fetchBytes)
  require(enablePfPipeline || outer.icacheParams.pfMSHRNum == 0,
    "When prefetch pipeline is disabled, icache pfMSHRNum must be 0")

  val bpd = Module(new BranchPredictor)
  bpd.io.f3_fire := true.B // 解耦前端 f3 预测不会被阻塞

  val ftq = Module(new FetchTargetQueue)
  ftq.io.f3_preds_enq.valid            := bpd.io.resp.f3_pred_valid
  ftq.io.f3_preds_enq.bits.preds_info  := bpd.io.resp.f3_preds_info
  ftq.io.f3_preds_enq.bits.fetch_info  := bpd.io.resp.f3_meta
  ftq.io.f3_preds_enq.bits.target_info := bpd.io.resp.f3_preds_target
  ftq.io.f3_ftq_idx_debug              := bpd.io.resp.f3_ftq_idx
  io.cpu.ft_tsrc                       <> ftq.io.ft_tsrc

  val icache = outer.icache.module
  icache.io.invalidate := io.cpu.flush_icache
  val tlb = Module(new TLB(true, log2Ceil(fetchBytes), TLBConfig(nTLBSets, nTLBWays)))
  io.ptw <> tlb.io.ptw
  io.cpu.perf.tlbMiss := io.ptw.req.fire
  io.cpu.perf.acquire := icache.io.perf.acquire

  // --------------------------------------------------------
  // **** NextPC Select (F0) ****
  //      Send request to ICache
  // --------------------------------------------------------

  // 这里额外加一级 s0_reg 要解决的问题是：f3 预测尝试写入 FTQ 时
  // FTQ 已满，应该如何处理？
  // 策略 1: 暂停 bpd f3 流水线，逐级反压。这样做的麻烦是，原本的
  // bpd 流水线是没有反压机制的，加 ready 信号有点麻烦，BOOM 本来的
  // 的前端实现也是假定了 bpd 这边没有暂停机制
  // 策略 2: bpd replay，让 f3 又回到 f1，重新预测。这样做也存在许
  // 多 corner case。根本问题在于，如果在 F3 预测要写入时才检查 FTQ
  // 是否满了，会导致 F1 和 F2 的 ftq idx 会可能会超 ifu 那边的 ftq
  // idx 1 圈，在环形指针的比较上会引入一些麻烦。
  // 目前是采取的最保守的策略，需要在发起 f1 预测或者 f1 取指时就分配
  // 合法的 ftq idx，这样保证 f3 预测或者预译码结果是一定能够写入的，虽
  // 然这可能带来一点性能损失

  // bpd s0 信号声明
  val s0_bpd_valid_reg   = RegInit(false.B)
  val s0_bpd_vpc_reg     = RegInit(0.U(vaddrBitsExtended.W))
  // 表示 s0 周期 ifu 要使用的 ftq idx，始终有效
  val s0_bpd_ftq_idx_reg = RegInit(FTQPtr(false.B, 0.U))
  val s0_bpd_ghist_reg   = RegInit((0.U).asTypeOf(new GlobalHistory))

  // s0 bpd vpc 的来源（仅用于 debug 打印）
  val bpdS0SrcRegDebug :: bpdS0SrcF1Debug :: bpdS0SrcF2Debug :: bpdS0SrcPredecodeDebug :: bpdS0SrcF3Debug :: bpdS0SrcSfenceDebug :: bpdS0SrcRedirectDebug :: Nil = Enum(7)
  val BPD_S0_SRC_REG_DEBUG       = bpdS0SrcRegDebug
  val BPD_S0_SRC_F1_DEBUG        = bpdS0SrcF1Debug
  val BPD_S0_SRC_F2_DEBUG        = bpdS0SrcF2Debug
  val BPD_S0_SRC_PREDECODE_DEBUG = bpdS0SrcPredecodeDebug
  val BPD_S0_SRC_F3_DEBUG        = bpdS0SrcF3Debug
  val BPD_S0_SRC_SFENCE_DEBUG    = bpdS0SrcSfenceDebug
  val BPD_S0_SRC_REDIRECT_DEBUG  = bpdS0SrcRedirectDebug
  val s0_bpd_tsrc_debug_reg = RegInit(BPD_S0_SRC_REG_DEBUG)

  val s0_bpd_valid   = WireInit(s0_bpd_valid_reg)
  val s0_bpd_vpc     = WireInit(s0_bpd_vpc_reg)
  // 表示 s0 周期 bpd 要使用的 ftq idx，始终有效
  val s0_bpd_ftq_idx = WireInit(s0_bpd_ftq_idx_reg)
  val s0_bpd_ghist   = WireInit(s0_bpd_ghist_reg)
  val s0_bpd_tsrc_debug    = WireInit(s0_bpd_tsrc_debug_reg)

  // prefetch s0 信号声明
  val s0_pf_valid_reg   = RegInit(false.B)
  val s0_pf_vpc_reg     = RegInit(0.U(vaddrBitsExtended.W))
  val s0_pf_ftq_idx_reg = RegInit(FTQPtr(false.B, 0.U))
  
  val s0_pf_valid       = WireDefault(s0_pf_valid_reg)
  val s0_pf_vpc         = WireDefault(s0_pf_vpc_reg)
  val s0_pf_ftq_idx     = WireDefault(s0_pf_ftq_idx_reg)

  // ifu s0 信号声明
  val s0_ifu_valid_reg   = RegInit(false.B)
  val s0_ifu_vpc_reg     = RegInit(0.U(vaddrBitsExtended.W))
  val s0_ifu_ftq_idx_reg = RegInit(FTQPtr(false.B, 0.U))
  val s0_ifu_tsrc_reg    = RegInit(BSRC_C)

  // s0 ifu vpc 的来源（仅用于 debug 打印）
  val ifuS0SrcRegDebug :: ifuS0SrcFtqDebug :: ifuS0SrcF1Debug :: ifuS0SrcF2Debug :: ifuS0SrcPredecodeDebug :: ifuS0SrcF3Debug :: ifuS0SrcS2ReplayDebug :: ifuS0SrcSfenceDebug :: ifuS0SrcRedirectDebug :: Nil = Enum(9)
  val IFU_S0_SRC_REG_DEBUG        = ifuS0SrcRegDebug
  val IFU_S0_SRC_FTQ_DEBUG        = ifuS0SrcFtqDebug
  val IFU_S0_SRC_F1_DEBUG         = ifuS0SrcF1Debug
  val IFU_S0_SRC_F2_DEBUG         = ifuS0SrcF2Debug
  val IFU_S0_SRC_PREDECODE_DEBUG  = ifuS0SrcPredecodeDebug
  val IFU_S0_SRC_F3_DEBUG         = ifuS0SrcF3Debug
  val IFU_S0_SRC_S2_REPLAY_DEBUG  = ifuS0SrcS2ReplayDebug
  val IFU_S0_SRC_SFENCE_DEBUG     = ifuS0SrcSfenceDebug
  val IFU_S0_SRC_REDIRECT_DEBUG   = ifuS0SrcRedirectDebug
  val s0_ifu_tsrc_debug_reg = RegInit(IFU_S0_SRC_REG_DEBUG)

  val s0_ifu_vpc   = WireInit(s0_ifu_vpc_reg)
  val s0_ftq_idx   = WireInit(s0_ifu_ftq_idx_reg)
  val s0_valid     = WireInit(s0_ifu_valid_reg)
  val s0_ifu_tsrc  = WireInit(s0_ifu_tsrc_reg)
  val s0_ifu_tsrc_debug = WireInit(s0_ifu_tsrc_debug_reg)
  val s0_is_replay = WireInit(false.B)
  val s0_is_real_replay = WireInit(false.B) // s0_is_replay 用于标识物理地址是否可用
  val s0_replay_is_ic_miss = WireInit(false.B)
  val s0_is_sfence = WireInit(false.B)
  val s0_replay_resp = Wire(new TLBResp(log2Ceil(fetchBytes)))
  val s0_replay_ppc  = Wire(UInt())

  // 考虑了 ftq full 后的 s0 valid 信号
  val s0_bpd_real_valid = Wire(Bool())
  val s0_pf_real_valid  = Wire(Bool())
  val s0_ifu_real_valid = Wire(Bool())

  // f3 预测重定向和预译码重定向可能同时发生，如果它们对应同一级流水线
  // 预译码重定向优先级更高
  val predecode_redirect = WireDefault(false.B)
  val bpd_f3_real_redirect = WireDefault(false.B)
  val s2_replay_happen = WireDefault(false.B)

  //Enable_PerfCounter_Support
  io.cpu.icache_valid_access := icache.io.icache_valid_access
  io.cpu.icache_hit := icache.io.resp.valid

  // Forward prefetch refill distance bucket from ICache to core
  io.cpu.pf_refill_dist_bucket := icache.io.pf_refill_dist_bucket

  // Forward prefetch accuracy signal from ICache to core
  io.cpu.pf_hit_success := icache.io.pf_hit_success

  // Dynamic UFTQ-AUR prefetch distance samples for perf counters
  io.cpu.dynamic_pf_dist_sample_count := false.B
  io.cpu.dynamic_pf_dist_sum := 0.U

  when (RegNext(reset.asBool) && !reset.asBool) {
    s0_valid   := true.B
    s0_ifu_vpc := io_reset_vector
    // s0_ftq_idx, s0_ifu_tsrc 使用 reset 后的初始值即可

    s0_bpd_valid := true.B
    s0_bpd_vpc   := io_reset_vector
    // s0_bpd_ftq_idx, s0_bpd_ghist 使用 reset 后的初始值即可
  }

  icache.io.req.valid     := s0_ifu_real_valid
  icache.io.req.bits.addr := s0_ifu_vpc
  // Prefetch s0: tag lookup request
  icache.io.s0_pf_valid   := s0_pf_real_valid
  icache.io.s0_pf_vaddr   := s0_pf_vpc

  // Compute prefetch-ahead-of-IFU distance and bucket in s0
  val pf_ifu_dist = Wire(UInt((log2Ceil(ftqSz) + 1).W))
  pf_ifu_dist := distanceBetween(s0_pf_ftq_idx, s0_ftq_idx)
  val pf_ahead_ifu = s0_pf_ftq_idx > s0_ftq_idx
  val s1_pf_ifu_dist = RegNext(pf_ifu_dist)
  val s1_pf_ahead_ifu = RegNext(pf_ahead_ifu)
  val pf_dist_bucket = Wire(UInt(3.W))
  when (pf_ifu_dist <= 1.U || !pf_ahead_ifu) {
    pf_dist_bucket := 0.U
  } .elsewhen (pf_ifu_dist <= 3.U) {
    pf_dist_bucket := 1.U
  } .elsewhen (pf_ifu_dist <= 5.U) {
    pf_dist_bucket := 2.U
  } .elsewhen (pf_ifu_dist <= 8.U) {
    pf_dist_bucket := 3.U
  } .elsewhen (pf_ifu_dist <= 12.U) {
    pf_dist_bucket := 4.U
  } .elsewhen (pf_ifu_dist <= 16.U) {
    pf_dist_bucket := 5.U
  } .otherwise {
    pf_dist_bucket := 6.U
  }
  icache.io.s0_pf_dist_bucket := pf_dist_bucket


  bpd.io.f0_req.valid      := s0_bpd_real_valid
  bpd.io.f0_req.bits.pc    := s0_bpd_vpc
  bpd.io.f0_req.bits.ghist := s0_bpd_ghist
  bpd.io.f0_req.bits.ftq_idx := s0_bpd_ftq_idx
  bpd.io.f0_req.bits.tsrc    := DontCare

  // --------------------------------------------------------
  // **** Prefetch (F1) ****
  //      Translate VPC
  // --------------------------------------------------------

  val trans_queue = Module(new ITLBLookup)
  val s1_is_sfence = RegNext(s0_is_sfence)

  val s1_pf_vpc       = RegNext(s0_pf_vpc)
  val s1_pf_ftq_idx   = RegNext(s0_pf_ftq_idx)
  val s1_pf_valid     = RegNext(s0_pf_real_valid, false.B)
  val f1_pf_clear     = WireInit(false.B)
  val s1_pf_replay    = RegInit(false.B)
  val s1_pf_replay_ppc = Reg(UInt(paddrBits.W))
  val s1_pf_replay_exp = Reg(Bool())

  if (enablePfPipeline) {
    tlb.io.req.valid      := (s1_pf_valid && !f1_pf_clear && !s1_pf_replay) || s1_is_sfence
    tlb.io.req.bits.vaddr := s1_pf_vpc
  }
  tlb.io.req.bits.cmd   := DontCare
  tlb.io.req.bits.passthrough := false.B
  tlb.io.req.bits.size  := log2Ceil(coreInstBytes * fetchWidth).U
  tlb.io.req.bits.v     := io.ptw.status.v
  tlb.io.req.bits.prv   := io.ptw.status.prv
  tlb.io.sfence         := RegNext(io.cpu.sfence)
  tlb.io.kill           := false.B

  val tlb_force_miss = WireDefault(false.B)
  val s1_pf_tlb_ok   = (!tlb.io.resp.miss && !tlb_force_miss)
  val s1_pf_ppc_valid = s1_pf_replay || s1_pf_tlb_ok
  val s1_pf_ppc_exp  = Mux(s1_pf_replay, s1_pf_replay_exp, tlb.io.resp.ae.inst || tlb.io.resp.pf.inst)

  if (enablePfPipeline) {
    trans_queue.io.enq.valid   := s1_pf_tlb_ok && !f1_pf_clear && s1_pf_valid && !s1_pf_replay
    trans_queue.io.enq.bits    := tlb.io.resp
    trans_queue.io.enq_ftq_idx := s1_pf_ftq_idx
  } else {
    trans_queue.io.enq.valid   := false.B
    trans_queue.io.enq.bits    := DontCare
    trans_queue.io.enq_ftq_idx := DontCare
  }

  val pfDistLimitWidth = if (enableUFTQAUR) {
    log2Ceil(Seq(ftqSz, uftqAurInitPfDist, uftqAurMinPfDist, uftqAurStep).max + uftqAurStep + 2)
  } else {
    math.max(1, log2Ceil(limitPfDist + 1))
  }
  val effectivePfDistLimit = if (enableUFTQAUR) {
    require(pfWindowSize > 0, "pfWindowSize must be positive when UFTQ-AUR is enabled")
    require(targetAccuracy >= 0 && targetAccuracy <= 100, "targetAccuracy must be in [0, 100]")
    require(uftqAurStep > 0, "uftqAurStep must be positive when UFTQ-AUR is enabled")
    require(uftqAurMinPfDist >= 0, "uftqAurMinPfDist must be non-negative")

    val maxPfDist = ftqSz - 1
    val minPfDist = math.min(uftqAurMinPfDist, maxPfDist)
    val initPfDist = math.max(minPfDist, math.min(uftqAurInitPfDist, maxPfDist))
    val targetHitCount = (targetAccuracy * pfWindowSize + 99) / 100
    val pfCounterWidth = log2Ceil(pfWindowSize + 1)

    val dynamicPfDist = RegInit(initPfDist.U(pfDistLimitWidth.W))
    val pfWindowCnt = RegInit(0.U(pfCounterWidth.W))
    val pfHitCnt = RegInit(0.U(pfCounterWidth.W))

    val pfRefill = icache.io.pf_refill_dist_bucket.valid
    val pfHit = icache.io.pf_hit_success
    val pfWindowDone = pfRefill && pfWindowCnt === (pfWindowSize - 1).U
    val pfHitCntInc = pfHitCnt +& pfHit.asUInt
    val nextPfHitCnt = Mux(pfHitCntInc > pfWindowSize.U,
      pfWindowSize.U(pfCounterWidth.W),
      pfHitCntInc(pfCounterWidth-1, 0))

    val maxPfDistU = maxPfDist.U(pfDistLimitWidth.W)
    val minPfDistU = minPfDist.U(pfDistLimitWidth.W)
    val stepU = uftqAurStep.U(pfDistLimitWidth.W)
    val incSum = dynamicPfDist +& stepU
    val incPfDist = Mux(incSum > maxPfDistU, maxPfDistU, incSum(pfDistLimitWidth-1, 0))
    val decClampThreshold = math.min(maxPfDist, minPfDist + uftqAurStep)
    val decPfDist = Mux(dynamicPfDist <= decClampThreshold.U(pfDistLimitWidth.W),
      minPfDistU,
      dynamicPfDist - stepU)
    val dynamicPfDistSample = if (pfDistLimitWidth >= 8) {
      dynamicPfDist(7, 0)
    } else {
      dynamicPfDist
    }

    when (pfWindowDone) {
      io.cpu.dynamic_pf_dist_sample_count := true.B
      io.cpu.dynamic_pf_dist_sum := dynamicPfDistSample
      dynamicPfDist := Mux(nextPfHitCnt >= targetHitCount.U, incPfDist, decPfDist)
      pfWindowCnt := 0.U
      pfHitCnt := 0.U
    } .otherwise {
      when (pfRefill) {
        pfWindowCnt := (pfWindowCnt + 1.U)(pfCounterWidth-1, 0)
      }
      when (pfHit) {
        pfHitCnt := nextPfHitCnt
      }
    }

    dynamicPfDist
  } else {
    limitPfDist.U(pfDistLimitWidth.W)
  }

  // s1_pf_can_go 表示 s1 不再需要保留：被 clear 或者 TLB 命中且 ICache 接收到了这一条 pf 请求
  val s1_pf_dist_exceeded = (effectivePfDistLimit > 0.U) && s1_pf_ahead_ifu && (s1_pf_ifu_dist > effectivePfDistLimit)
  val s1_pf_can_advance = icache.io.s1_pf_can_advance && !s1_pf_dist_exceeded
  val s1_pf_can_go = s1_pf_ppc_valid && s1_pf_can_advance

  // Drive prefetch-related inputs to ICache s1 stage
  icache.io.s1_pf_valid     := s1_pf_valid && !s1_pf_dist_exceeded
  icache.io.s1_pf_ppc       := Mux(s1_pf_replay, s1_pf_replay_ppc, tlb.io.resp.paddr)
  icache.io.s1_pf_ppc_valid := s1_pf_ppc_valid
  icache.io.s1_pf_ppc_exp   := s1_pf_ppc_exp
  icache.io.s1_pf_clear     := f1_pf_clear
  icache.io.s1_pf_ftq_idx   := s1_pf_ftq_idx

  // --------------------------------------------------------
  // **** ICache Access (F1) ****
  //      Translate VPC
  // --------------------------------------------------------
  val s1_vpc       = RegNext(s0_ifu_vpc)
  val s1_ftq_idx   = RegNext(s0_ftq_idx)
  val s1_valid     = RegNext(s0_ifu_real_valid, false.B)
  val s1_ifu_tsrc  = RegNext(s0_ifu_tsrc)
  val s1_ghist     = RegNext(s0_bpd_ghist)
  val s1_is_replay = RegNext(s0_is_replay)
  val s2_ic_miss_stall_cycles = RegInit(0.U(12.W))
  val s1_ic_miss_stall_cycles = RegInit(0.U(12.W))
  when (!s0_is_real_replay) {
    s1_ic_miss_stall_cycles := 0.U
  } .elsewhen (!s0_replay_is_ic_miss) {
    s1_ic_miss_stall_cycles := s2_ic_miss_stall_cycles
  } .otherwise {
    s1_ic_miss_stall_cycles := Mux(
      s2_ic_miss_stall_cycles === ((1 << 12) - 1).U,
      ((1 << 12) - 1).U,
      s2_ic_miss_stall_cycles + 1.U)
  }
  s2_ic_miss_stall_cycles := s1_ic_miss_stall_cycles
  val f1_clear     = WireInit(false.B)
  val bpd_f1_clear = WireInit(false.B)

  if (enablePfPipeline) {
    // ifu 这边不再访问 tlb，从 trans_queue 中拿翻译结果
    trans_queue.io.deq.ready := s1_valid && !s1_is_replay && !f1_clear
  } else {
    // 预取流水线关闭时，ifu 直接访问 tlb
    tlb.io.req.valid      := (s1_valid && !s1_is_replay && !f1_clear) || s1_is_sfence
    tlb.io.req.bits.vaddr := s1_vpc
    trans_queue.io.deq.ready := false.B
  }
  val s1_tlb_miss = if (enablePfPipeline) {
    !s1_is_replay && !trans_queue.io.deq.valid
  } else {
    !s1_is_replay && tlb.io.resp.miss
  }
  val s1_tlb_resp = if (enablePfPipeline) {
    Mux(s1_is_replay, RegNext(s0_replay_resp), trans_queue.io.deq.bits)
  } else {
    Mux(s1_is_replay, RegNext(s0_replay_resp), tlb.io.resp)
  }
  val s1_ppc = if (enablePfPipeline) {
    Mux(s1_is_replay, RegNext(s0_replay_ppc), trans_queue.io.deq.bits.paddr)
  } else {
    Mux(s1_is_replay, RegNext(s0_replay_ppc), tlb.io.resp.paddr)
  }

  icache.io.s1_paddr := s1_ppc
  icache.io.s1_kill  := s1_tlb_miss || f1_clear

  // TODO: 重新考虑这里的逻辑
  io.cpu.itlb_valid_access := (s1_valid && !s1_is_replay && !f1_clear) || s1_is_sfence
  io.cpu.itlb_hit := tlb.io.req.valid && !s1_tlb_miss

  // --------------------------------------------------------
  // **** ICache Response (F2) ****
  // --------------------------------------------------------

  val s2_valid = RegNext(s1_valid && !f1_clear, false.B)
  val s2_ftq_idx   = RegNext(s1_ftq_idx)
  val s2_ifu_tsrc  = RegNext(s1_ifu_tsrc)
  val s2_vpc   = RegNext(s1_vpc)
  val s2_ppc  = RegNext(s1_ppc)
  val f2_clear = WireInit(false.B)
  val bpd_f2_clear = WireInit(false.B)
  val s2_tlb_resp = RegNext(s1_tlb_resp)
  val s2_tlb_miss = RegNext(s1_tlb_miss)
  val s2_is_replay = RegNext(s1_is_replay) && s2_valid
  // 当 tlb miss 时 ae.inst 和 pf.inst 为 false
  val s2_xcpt = s2_valid && (s2_tlb_resp.ae.inst || s2_tlb_resp.pf.inst)
  val f3_ready = Wire(Bool())

  icache.io.s2_kill := s2_xcpt

  val f3_enq_fire = Wire(Bool())
  when ((s2_valid && !icache.io.resp.valid) ||
        (s2_valid && icache.io.resp.valid && !f3_ready)) {
    s2_replay_happen := true.B
  } 
  // Drive s2 replay perf signals
  io.cpu.s2_replay_total    := s2_replay_happen
  io.cpu.s2_replay_itlb_miss := s2_replay_happen && s2_tlb_miss
  io.cpu.s2_replay_ic_miss   := s2_replay_happen && !s2_tlb_miss && !icache.io.resp.valid
  s0_replay_resp := s2_tlb_resp
  s0_replay_ppc  := s2_ppc

  bpd.io.f1_clear := bpd_f1_clear
  bpd.io.f2_clear := bpd_f2_clear

  ftq.io.s2_ftq_idx := s2_ftq_idx

  // --------------------------------------------------------
  // **** F3 ****
  // --------------------------------------------------------
  val f3_clear = WireInit(false.B)
  val f3 = withReset(reset.asBool || f3_clear) {
    Module(new Queue(new FrontendResp, 1, pipe=true, flow=false)) }

  // Queue up the bpd resp as well, incase f4 backpressures f3
  // This is "flow" because the response (enq) arrives in f3, not f2
  val f3_bpd_resp = withReset(reset.asBool || f3_clear) {
    Module(new Queue(new BranchPredBundleWithGHist, 1, pipe=true, flow=true)) }




  val f4_ready = Wire(Bool())
  f3_ready := f3.io.enq.ready
  f3.io.enq.valid   := (s2_valid && !f2_clear &&
    (icache.io.resp.valid || ((s2_tlb_resp.ae.inst || s2_tlb_resp.pf.inst) && !s2_tlb_miss))
  )
  f3.io.enq.bits.ftq_idx := s2_ftq_idx
  f3.io.enq.bits.pc := s2_vpc
  f3.io.enq.bits.data  := Mux(s2_xcpt, 0.U, icache.io.resp.bits.data)
  f3.io.enq.bits.mask := fetchMask(s2_vpc)
  f3.io.enq.bits.xcpt := s2_tlb_resp
  f3.io.enq.bits.tsrc := s2_ifu_tsrc
  f3.io.enq.bits.ic_miss_stall_cycles := s2_ic_miss_stall_cycles

  f3_enq_fire := f3.io.enq.fire
  icache.io.f3_enq_fire := f3_enq_fire

  // The BPD resp comes in f3
  f3_bpd_resp.io.enq.valid := f3.io.deq.valid && RegNext(f3.io.enq.ready)
  f3_bpd_resp.io.enq.bits.pc  := ftq.io.s3_preds_info.pc_debug
  f3_bpd_resp.io.enq.bits.preds.br_taken := ftq.io.s3_preds_info.preds_info.br_taken
  f3_bpd_resp.io.enq.bits.preds.jal_target := ftq.io.s3_preds_info.preds_info.jal_target
  f3_bpd_resp.io.enq.bits.preds.jal_targets_debug := DontCare
  f3_bpd_resp.io.enq.bits.preds.ras_top := ftq.io.s3_preds_info.ras_top
  f3_bpd_resp.io.enq.bits.preds.ras_idx := ftq.io.s3_preds_info.ras_idx
  f3_bpd_resp.io.enq.bits.preds.btb_hits := ftq.io.s3_preds_info.preds_info.btb_hits
  f3_bpd_resp.io.enq.bits.ghist_update_type := ftq.io.s3_preds_info.preds_info.ghist_update_type
  f3_bpd_resp.io.enq.bits.meta  := DontCare
  f3_bpd_resp.io.enq.bits.ghist := ftq.io.s3_preds_info.ghist
  f3_bpd_resp.io.enq.bits.fsrc := ftq.io.s3_preds_info.preds_info.fsrc
  f3_bpd_resp.io.enq.bits.tsrc := ftq.io.s3_preds_info.preds_info.tsrc
  f3_bpd_resp.io.enq.bits.check_target := ftq.io.s3_preds_info.check_target
  f3_bpd_resp.io.enq.bits.bpd_target := ftq.io.s3_preds_info.bpd_target

  f3.io.deq.ready := f4_ready
  f3_bpd_resp.io.deq.ready := f4_ready


  val f3_imemresp     = f3.io.deq.bits
  val f3_bank_mask    = bankMask(f3_imemresp.pc)
  val f3_data         = f3_imemresp.data
  val f3_aligned_pc   = bankAlign(f3_imemresp.pc)
  val f3_is_last_bank_in_block = isLastBankInBlock(f3_aligned_pc)
  val f3_is_rvc       = Wire(Vec(fetchWidth, Bool()))
  val f3_redirects    = Wire(Vec(fetchWidth, Bool()))
  val f3_targs        = Wire(Vec(fetchWidth, UInt(vaddrBitsExtended.W)))
  val f3_cfi_types    = Wire(Vec(fetchWidth, UInt(CFI_SZ.W)))
  val f3_shadowed_mask = Wire(Vec(fetchWidth, Bool()))
  val f3_fetch_bundle = Wire(new FetchBundle)
  val f3_mask         = Wire(Vec(fetchWidth, Bool()))
  val f3_br_mask      = Wire(Vec(fetchWidth, Bool()))
  val f3_call_mask    = Wire(Vec(fetchWidth, Bool()))
  val f3_ret_mask     = Wire(Vec(fetchWidth, Bool()))
  val f3_npc_plus4_mask = Wire(Vec(fetchWidth, Bool()))
  val f3_btb_mispredicts = Wire(Vec(fetchWidth, Bool()))
  f3_fetch_bundle.mask := f3_mask.asUInt
  f3_fetch_bundle.br_mask := f3_br_mask.asUInt
  f3_fetch_bundle.pc := f3_imemresp.pc
  f3_fetch_bundle.ftq_idx := f3.io.deq.bits.ftq_idx
  f3_fetch_bundle.xcpt_pf_if := f3_imemresp.xcpt.pf.inst
  f3_fetch_bundle.xcpt_ae_if := f3_imemresp.xcpt.ae.inst
  f3_fetch_bundle.fsrc := f3_bpd_resp.io.deq.bits.fsrc
  f3_fetch_bundle.tsrc := f3_imemresp.tsrc
  f3_fetch_bundle.ic_miss_stall_cycles := f3_imemresp.ic_miss_stall_cycles
  f3_fetch_bundle.shadowed_mask := f3_shadowed_mask

  // Tracks trailing 16b of previous fetch packet
  val f3_prev_half    = Reg(UInt(16.W))
  // Tracks if last fetchpacket contained a half-inst
  val f3_prev_is_half = RegInit(false.B)

  // 现在的译码逻辑的输入不包含 f3 预测
  // 它的输出包括 5 个 predecode_xxx 信号，f3_is_rvc, f3_npc_plus4_mask 信号 
  // f3_fetch_bundle 的 insts, exp_insts, edge_inst 信号
  // 以及 bank_prev_is_half, bank_prev_half 信号
  // 预译码输出的 bank_prev_is_half 尚未考虑分支跳转的影响 

  // 表示该指令是否有效（考虑了 fetch pc 和压缩指令的影响，没考虑分支指令跳转）
  val predecode_valid = Wire(Vec(fetchWidth, Bool()))
  val predecode_is_call = Wire(Vec(fetchWidth, Bool()))
  val predecode_is_ret  = Wire(Vec(fetchWidth, Bool()))
  val predecode_cfi_type = Wire(Vec(fetchWidth, UInt(CFI_SZ.W)))
  val predecode_targets = Wire(Vec(fetchWidth, UInt(vaddrBitsExtended.W)))

  require(fetchWidth >= 4) // Logic gets kind of annoying with fetchWidth = 2
  def isRVC(inst: UInt) = (inst(1,0) =/= 3.U)
  var bank_prev_is_half = f3_prev_is_half
  var bank_prev_half    = f3_prev_half
  var last_inst = 0.U(16.W)
  for (b <- 0 until nBanks) {
    val bank_data  = f3_data((b+1)*bankWidth*16-1, b*bankWidth*16)
    val bank_mask  = Wire(Vec(bankWidth, Bool()))
    val bank_insts = Wire(Vec(bankWidth, UInt(32.W)))

    for (w <- 0 until bankWidth) {
      val i = (b * bankWidth) + w

      val valid = Wire(Bool())
      val bpu = Module(new BreakpointUnit(nBreakpoints))
      bpu.io.status   := io.cpu.status
      bpu.io.bp       := io.cpu.bp
      bpu.io.ea       := DontCare
      bpu.io.mcontext := io.cpu.mcontext
      bpu.io.scontext := io.cpu.scontext

      val brsigs = Wire(new BranchDecodeSignals)
      if (w == 0) {
        val inst0 = Cat(bank_data(15,0), f3_prev_half)
        val inst1 = bank_data(31,0)
        val exp_inst0 = ExpandRVC(inst0)
        val exp_inst1 = ExpandRVC(inst1)
        val pc0 = (f3_aligned_pc + (i << log2Ceil(coreInstBytes)).U - 2.U)
        val pc1 = (f3_aligned_pc + (i << log2Ceil(coreInstBytes)).U)

        val bpd_decoder0 = Module(new BranchDecode)
        bpd_decoder0.io.inst := exp_inst0
        bpd_decoder0.io.pc   := pc0
        val bpd_decoder1 = Module(new BranchDecode)
        bpd_decoder1.io.inst := exp_inst1
        bpd_decoder1.io.pc   := pc1

        when (bank_prev_is_half) {
          bank_insts(w)                := inst0
          f3_fetch_bundle.insts(i)     := inst0
          f3_fetch_bundle.exp_insts(i) := exp_inst0
          bpu.io.pc                    := pc0
          brsigs                       := bpd_decoder0.io.out
          f3_fetch_bundle.edge_inst(b) := true.B
          if (b > 0) {
            val inst0b     = Cat(bank_data(15,0), last_inst)
            val exp_inst0b = ExpandRVC(inst0b)
            val bpd_decoder0b = Module(new BranchDecode)
            bpd_decoder0b.io.inst := exp_inst0b
            bpd_decoder0b.io.pc   := pc0

            when (f3_bank_mask(b-1)) {
              bank_insts(w)                := inst0b
              f3_fetch_bundle.insts(i)     := inst0b
              f3_fetch_bundle.exp_insts(i) := exp_inst0b
              brsigs                       := bpd_decoder0b.io.out
            }
          }
        } .otherwise {
          bank_insts(w)                := inst1
          f3_fetch_bundle.insts(i)     := inst1
          f3_fetch_bundle.exp_insts(i) := exp_inst1
          bpu.io.pc                    := pc1
          brsigs                       := bpd_decoder1.io.out
          f3_fetch_bundle.edge_inst(b) := false.B
        }
        valid := true.B
      } else {
        val inst = Wire(UInt(32.W))
        val exp_inst = ExpandRVC(inst)
        val pc = f3_aligned_pc + (i << log2Ceil(coreInstBytes)).U
        val bpd_decoder = Module(new BranchDecode)
        bpd_decoder.io.inst := exp_inst
        bpd_decoder.io.pc   := pc

        bank_insts(w)                := inst
        f3_fetch_bundle.insts(i)     := inst
        f3_fetch_bundle.exp_insts(i) := exp_inst
        bpu.io.pc                    := pc
        brsigs                       := bpd_decoder.io.out
        if (w == 1) {
          // Need special case since 0th instruction may carry over the wrap around
          inst  := bank_data(47,16)
          valid := bank_prev_is_half || !(bank_mask(0) && !isRVC(bank_insts(0)))
        } else if (w == bankWidth - 1) {
          inst  := Cat(0.U(16.W), bank_data(bankWidth*16-1,(bankWidth-1)*16))
          valid := !((bank_mask(w-1) && !isRVC(bank_insts(w-1))) ||
            !isRVC(inst))
        } else {
          inst  := bank_data(w*16+32-1,w*16)
          valid := !(bank_mask(w-1) && !isRVC(bank_insts(w-1)))
        }
      }

      f3_is_rvc(i) := isRVC(bank_insts(w))
      bank_mask(w) := f3.io.deq.valid && f3_imemresp.mask(i) && valid
      predecode_valid(i) := bank_mask(w)
      predecode_is_call(i) := brsigs.is_call
      predecode_is_ret (i) := brsigs.is_ret
      predecode_cfi_type(i) := brsigs.cfi_type
      predecode_targets(i) := brsigs.target

      f3_npc_plus4_mask(i) := (if (w == 0) {
        !f3_is_rvc(i) && !bank_prev_is_half
      } else {
        !f3_is_rvc(i)
      })
      val offset_from_aligned_pc = (
        (i << 1).U((log2Ceil(icBlockBytes)+1).W) +
        brsigs.sfb_offset.bits -
        Mux(bank_prev_is_half && (w == 0).B, 2.U, 0.U)
      )
      val lower_mask = Wire(UInt((2*fetchWidth).W))
      val upper_mask = Wire(UInt((2*fetchWidth).W))
      lower_mask := UIntToOH(i.U)
      upper_mask := UIntToOH(offset_from_aligned_pc(log2Ceil(fetchBytes)+1,1)) << Mux(f3_is_last_bank_in_block, bankWidth.U, 0.U)

      f3_fetch_bundle.sfbs(i) := (
        f3_mask(i) &&
        brsigs.sfb_offset.valid &&
        (offset_from_aligned_pc <= Mux(f3_is_last_bank_in_block, (fetchBytes+bankBytes).U,(2*fetchBytes).U))
      )
      f3_fetch_bundle.sfb_masks(i)       := ~MaskLower(lower_mask) & ~MaskUpper(upper_mask)
      f3_fetch_bundle.shadowable_mask(i) := (!(f3_fetch_bundle.xcpt_pf_if || f3_fetch_bundle.xcpt_ae_if || bpu.io.debug_if || bpu.io.xcpt_if) &&
                                             f3_bank_mask(b) &&
                                             (brsigs.shadowable || !f3_mask(i)))
      f3_fetch_bundle.sfb_dests(i)       := offset_from_aligned_pc

      f3_fetch_bundle.bp_debug_if_oh(i) := bpu.io.debug_if
      f3_fetch_bundle.bp_xcpt_if_oh (i) := bpu.io.xcpt_if

    }
    last_inst = bank_insts(bankWidth-1)(15,0)
    bank_prev_is_half = Mux(f3_bank_mask(b),
      (!(bank_mask(bankWidth-2) && !isRVC(bank_insts(bankWidth-2))) && !isRVC(last_inst)),
      bank_prev_is_half)
    bank_prev_half    = Mux(f3_bank_mask(b),
      last_inst(15,0),
      bank_prev_half)
  }

  var redirect_found = false.B
  for (i <- 0 until fetchWidth) {
      // Redirect if
      //  1) its a JAL/JALR (unconditional)
      //  2) the BPD believes this is a branch and says we should take it
      f3_redirects(i)    := predecode_valid(i) && (
        predecode_cfi_type(i) === CFI_JAL || predecode_cfi_type(i) === CFI_JALR ||
        (predecode_cfi_type(i) === CFI_BR && f3_bpd_resp.io.deq.bits.preds.br_taken(i) && useBPD.B)
      )
      f3_mask  (i) := predecode_valid(i) && !redirect_found
      f3_targs (i) := Mux(predecode_cfi_type(i) === CFI_JALR && !disablePostJalrCorrection.B,
        f3_bpd_resp.io.deq.bits.preds.jal_target,
        predecode_targets(i))

      // 如果从 fetch pc 开始的非指令或者非分支指令被预测为分支指令，清空该 btb 表项
      f3_btb_mispredicts(i) := (predecode_cfi_type(i) === CFI_X || !predecode_valid(i)) &&
                                f3_bpd_resp.io.deq.bits.preds.btb_hits(i) && f3_imemresp.mask(i)

      f3_br_mask(i)   := f3_mask(i) && predecode_cfi_type(i) === CFI_BR
      f3_cfi_types(i) := predecode_cfi_type(i)
      f3_call_mask(i) := predecode_is_call(i)
      f3_ret_mask(i)  := predecode_is_ret(i)

      redirect_found = redirect_found || f3_redirects(i)
  }


  f3_fetch_bundle.cfi_type      := f3_cfi_types(f3_fetch_bundle.cfi_idx.bits)
  f3_fetch_bundle.cfi_is_call   := f3_call_mask(f3_fetch_bundle.cfi_idx.bits)
  f3_fetch_bundle.cfi_is_ret    := f3_ret_mask (f3_fetch_bundle.cfi_idx.bits)
  f3_fetch_bundle.cfi_npc_plus4 := f3_npc_plus4_mask(f3_fetch_bundle.cfi_idx.bits)

  f3_fetch_bundle.ghist    := f3_bpd_resp.io.deq.bits.ghist
  // 忽略 f1, f2, f3 ghist 的 ras_idx 字段
  f3_fetch_bundle.ghist.ras_idx := f3_bpd_resp.io.deq.bits.preds.ras_idx
  f3_fetch_bundle.bpd_meta := f3_bpd_resp.io.deq.bits.meta
  f3_fetch_bundle.btb_mispredicts := f3_btb_mispredicts.asUInt

  f3_fetch_bundle.end_half.valid := bank_prev_is_half
  f3_fetch_bundle.end_half.bits  := bank_prev_half

  when (f3.io.deq.fire) {
    f3_prev_is_half := bank_prev_is_half
    f3_prev_half    := bank_prev_half
    assert(f3_bpd_resp.io.deq.bits.pc === f3_fetch_bundle.pc)
  }

  when (f3_clear) {
    f3_prev_is_half := false.B
  }

  f3_fetch_bundle.cfi_idx.valid := f3_redirects.reduce(_||_)
  f3_fetch_bundle.cfi_idx.bits  := PriorityEncoder(f3_redirects)

  f3_fetch_bundle.ras_top := f3_bpd_resp.io.deq.bits.preds.ras_top
  // Redirect earlier stages only if the later stage
  // can consume this packet

  val f3_has_redirect = f3_redirects.reduce(_||_)
  val f3_cfi_is_ret = f3_fetch_bundle.cfi_is_ret && f3_fetch_bundle.cfi_idx.valid
  val f3_redirect_idx = PriorityEncoder(f3_redirects)
  val f3_bpd_predicted_target = f3_bpd_resp.io.deq.bits.bpd_target
  val f3_is_jalr = f3_fetch_bundle.cfi_idx.valid && f3_fetch_bundle.cfi_type === CFI_JALR
  val f3_use_bpd_target =
    (f3_cfi_is_ret && disablePostRetCorrection.B) ||
    (f3_is_jalr && !f3_cfi_is_ret && disablePostJalrCorrection.B)
  val f3_predicted_target = Mux(f3_has_redirect,
    Mux(f3_use_bpd_target,
      f3_bpd_predicted_target,
      Mux(f3_cfi_is_ret && useBPD.B && useRAS.B,
        f3_bpd_resp.io.deq.bits.preds.ras_top,
        f3_targs(f3_redirect_idx)
      )),
    nextFetch(f3_fetch_bundle.pc)
  )

  f3_fetch_bundle.next_pc       := f3_predicted_target
  val (f3_predicted_ghist, f3_pred_ghist_update_type) = f3_fetch_bundle.ghist.update(
    f3_fetch_bundle.br_mask,
    f3_fetch_bundle.cfi_idx.valid,
    f3_fetch_bundle.br_mask(f3_fetch_bundle.cfi_idx.bits),
    f3_fetch_bundle.cfi_idx.bits,
    f3_fetch_bundle.cfi_idx.valid,
    f3_fetch_bundle.pc,
    f3_fetch_bundle.cfi_is_call,
    f3_fetch_bundle.cfi_is_ret
  )

  // 如果后端重定向引发 ras top idx 的修正和 f3_fire 引发 call/ret 指令修改 ras 内容
  // 可以同时发生，互不干扰，因此这里没有检查 f3_clear 信号。下一周期 ftq 传来 ras 内容的
  // 修正请求时，f3_fire 必为 false
  bpd.io.predecode_ras_update_valid := false.B
  bpd.io.predecode_ras_update_addr  := DontCare
  bpd.io.predecode_ras_update_idx   := DontCare
  bpd.io.predecode_ras_top_update_valid := false.B
  bpd.io.predecode_ras_top_update_idx   := DontCare

  bpd.io.backend_ras_update_valid := false.B
  bpd.io.backend_ras_update_addr  := DontCare
  bpd.io.backend_ras_update_idx   := DontCare
  bpd.io.backend_ras_top_update_valid := false.B
  bpd.io.backend_ras_top_update_idx := DontCare

  // Deferred predecode call RAS restoration: after a predecode redirect with a call,
  // on the next cycle we restore the old ras_top value at the original ras_idx.
  // (The pointer was already bumped in the first cycle, so only content needs restoring.)
  val predecode_call_ras_restore_valid = RegInit(false.B)
  val predecode_call_ras_restore_idx   = Reg(UInt(log2Ceil(nRasEntries).W))
  val predecode_call_ras_restore_addr  = Reg(UInt(vaddrBitsExtended.W))
  predecode_call_ras_restore_valid := false.B
  when (predecode_call_ras_restore_valid) {
    bpd.io.predecode_ras_update_valid := true.B
    bpd.io.predecode_ras_update_idx   := predecode_call_ras_restore_idx
    bpd.io.predecode_ras_update_addr  := predecode_call_ras_restore_addr
  }



  val f3_ghist_all_zero = f3_fetch_bundle.ghist === (0.U).asTypeOf(new GlobalHistory)
  val shift_zero_or_no_shift_f3 = f3_pred_ghist_update_type(1) === 0.U &&
                                  f3_bpd_resp.io.deq.bits.ghist_update_type(1) === 0.U
  val f3_correct_ghist = !(f3_ghist_all_zero && shift_zero_or_no_shift_f3) &&
                          f3_pred_ghist_update_type =/= f3_bpd_resp.io.deq.bits.ghist_update_type &&
                          enableGHistStallRepair.B
  val f3_correct_target = f3_predicted_target =/= f3_bpd_resp.io.deq.bits.check_target
  val itlb_exception = f3_fetch_bundle.xcpt_pf_if || f3_fetch_bundle.xcpt_ae_if

  val f3_bp_check_happened = RegInit(false.B)

  when (f3.io.deq.valid && !f3_bp_check_happened) {
    predecode_redirect := f3_correct_ghist || f3_correct_target || itlb_exception
    when(predecode_redirect) {
      f3_fetch_bundle.fsrc := BSRC_3
    }
    when (predecode_redirect || (bpd.io.resp.f3_pred_valid &&
          f3_fetch_bundle.ftq_idx === bpd.io.resp.f3_ftq_idx)) {
      // 正常来讲需要将 predecode_redirect 和 predecode suppress f3 preds 的情况
      // 区分一下，但对于后者，没有预测错时，更新也不会造成任何影响？
      when (f3_fetch_bundle.cfi_is_call && f3_fetch_bundle.cfi_idx.valid) {
        // 预译码检测到 call 指令时，ras top idx 更新和 ras top 内容更新同时发生
        // 本周期：将新的 call 返回地址写入 ras_idx+1，并将 ras top 指针移到 ras_idx+1
        bpd.io.predecode_ras_top_update_valid := true.B
        bpd.io.predecode_ras_top_update_idx := WrapInc(f3_bpd_resp.io.deq.bits.preds.ras_idx, nRasEntries)
        bpd.io.predecode_ras_update_valid := true.B
        bpd.io.predecode_ras_update_idx := WrapInc(f3_bpd_resp.io.deq.bits.preds.ras_idx, nRasEntries)
        bpd.io.predecode_ras_update_addr := f3_aligned_pc + (f3_fetch_bundle.cfi_idx.bits << 1) + Mux(
                                            f3_fetch_bundle.cfi_npc_plus4, 4.U, 2.U)
        // 下一周期：恢复原来 ras_idx 位置的 ras_top 值（可能被投机执行覆盖）
        predecode_call_ras_restore_valid := true.B
        predecode_call_ras_restore_idx   := f3_bpd_resp.io.deq.bits.preds.ras_idx
        predecode_call_ras_restore_addr  := f3_bpd_resp.io.deq.bits.preds.ras_top
      } .elsewhen(f3_fetch_bundle.cfi_is_ret && f3_fetch_bundle.cfi_idx.valid) {
        // 预译码检测到 ret 指令时，更新 ras top idx
        bpd.io.predecode_ras_top_update_valid := true.B 
        bpd.io.predecode_ras_top_update_idx := WrapDec(f3_bpd_resp.io.deq.bits.preds.ras_idx, nRasEntries)
        bpd.io.predecode_ras_update_valid := true.B
        bpd.io.predecode_ras_update_idx :=f3_bpd_resp.io.deq.bits.preds.ras_idx
        bpd.io.predecode_ras_update_addr := f3_bpd_resp.io.deq.bits.preds.ras_top  
      } .otherwise {
        bpd.io.predecode_ras_top_update_valid := true.B 
        bpd.io.predecode_ras_top_update_idx := f3_bpd_resp.io.deq.bits.preds.ras_idx      
        bpd.io.predecode_ras_update_valid := true.B
        bpd.io.predecode_ras_update_idx :=f3_bpd_resp.io.deq.bits.preds.ras_idx
        bpd.io.predecode_ras_update_addr := f3_bpd_resp.io.deq.bits.preds.ras_top         
      }
    }
    f3_bp_check_happened := true.B
  }

  when (f3.io.deq.valid && f4_ready) {
    when (f3_redirects.reduce(_||_)) {
      f3_prev_is_half := false.B
    }
  }

  when (f3.io.enq.valid && f3_ready) {
    f3_bp_check_happened := false.B
  }

  ftq.io.predecode_redirect.valid := predecode_redirect
  ftq.io.predecode_redirect.bits  := f3_fetch_bundle.ftq_idx

  // When f3 finds a btb mispredict, queue up a bpd correction update
  val f4_btb_corrections = Module(new Queue(new BranchPredictionUpdate, 2))
  f4_btb_corrections.io.enq.valid := false.B && enableBTBFastRepair.B
  f4_btb_corrections.io.enq.bits  := DontCare
  f4_btb_corrections.io.enq.bits.is_mispredict_update := false.B
  f4_btb_corrections.io.enq.bits.is_repair_update     := false.B
  f4_btb_corrections.io.enq.bits.btb_mispredicts      := f3_btb_mispredicts.asUInt
  f4_btb_corrections.io.enq.bits.pc                   := f3_fetch_bundle.pc
  f4_btb_corrections.io.enq.bits.ghist                := f3_fetch_bundle.ghist
  f4_btb_corrections.io.enq.bits.meta                 := f3_fetch_bundle.bpd_meta


  // -------------------------------------------------------
  // **** F4 ****
  // -------------------------------------------------------
  val f4_clear = WireInit(false.B)
  val f4 = withReset(reset.asBool || f4_clear) {
    Module(new Queue(new FetchBundle, 1, pipe=true, flow=false))}

  val fb  = Module(new FetchBuffer)

  // When we mispredict, we need to repair

  // Deal with sfbs
  val f4_shadowable_masks = VecInit((0 until fetchWidth) map { i =>
     f4.io.deq.bits.shadowable_mask.asUInt |
    ~f4.io.deq.bits.sfb_masks(i)(fetchWidth-1,0)
  })
  val f3_shadowable_masks = VecInit((0 until fetchWidth) map { i =>
    Mux(f4.io.enq.valid, f4.io.enq.bits.shadowable_mask.asUInt, 0.U) |
    ~f4.io.deq.bits.sfb_masks(i)(2*fetchWidth-1,fetchWidth)
  })
  val f4_sfbs = VecInit((0 until fetchWidth) map { i =>
    enableSFBOpt.B &&
    ((~f4_shadowable_masks(i) === 0.U) &&
     (~f3_shadowable_masks(i) === 0.U) &&
     f4.io.deq.bits.sfbs(i) &&
     !(f4.io.deq.bits.cfi_idx.valid && f4.io.deq.bits.cfi_idx.bits === i.U) &&
      Mux(f4.io.deq.bits.sfb_dests(i) === 0.U,
        !bank_prev_is_half,
      Mux(f4.io.deq.bits.sfb_dests(i) === fetchBytes.U,
        !f4.io.deq.bits.end_half.valid,
        true.B)
      )

     )
  })
  val f4_sfb_valid = f4_sfbs.reduce(_||_) && f4.io.deq.valid
  val f4_sfb_idx   = PriorityEncoder(f4_sfbs)
  val f4_sfb_mask  = f4.io.deq.bits.sfb_masks(f4_sfb_idx)
  // If we have a SFB, wait for next fetch to be available in f3
  val f4_delay = Wire(Bool())
  f4_delay     := (
    f4.io.deq.bits.sfbs.reduce(_||_) &&
    !f4.io.deq.bits.cfi_idx.valid &&
    !f4.io.enq.valid &&
    !f4.io.deq.bits.xcpt_pf_if &&
    !f4.io.deq.bits.xcpt_ae_if
  )
  when (f4_sfb_valid) {
    f3_shadowed_mask := f4_sfb_mask(2*fetchWidth-1,fetchWidth).asBools
  } .otherwise {
    f3_shadowed_mask := VecInit(0.U(fetchWidth.W).asBools)
  }

  f4_ready := f4.io.enq.ready
  f4.io.enq.valid := f3.io.deq.valid && !f3_clear
  f4.io.enq.bits  := f3_fetch_bundle
  f4.io.deq.ready := fb.io.enq.ready && !f4_delay

  io.cpu.ic_miss_stall_seq       := 0.U
  io.cpu.ic_miss_stall_cond      := 0.U
  io.cpu.ic_miss_stall_jal       := 0.U
  io.cpu.ic_miss_stall_jalr      := 0.U
  io.cpu.ic_miss_stall_ret       := 0.U
  io.cpu.ic_miss_stall_exception := 0.U

  when (ftq.io.update_miss_counter) {
    switch (ftq.io.last_commit_cfi_type) {
      is (IC_MISS_SEQ)       { io.cpu.ic_miss_stall_seq       := ftq.io.commit_ic_stall_cycles }
      is (IC_MISS_COND)      { io.cpu.ic_miss_stall_cond      := ftq.io.commit_ic_stall_cycles }
      is (IC_MISS_JAL)       { io.cpu.ic_miss_stall_jal       := ftq.io.commit_ic_stall_cycles }
      is (IC_MISS_JALR)      { io.cpu.ic_miss_stall_jalr      := ftq.io.commit_ic_stall_cycles }
      is (IC_MISS_RET)       { io.cpu.ic_miss_stall_ret       := ftq.io.commit_ic_stall_cycles }
      is (IC_MISS_EXCEPTION) { io.cpu.ic_miss_stall_exception := ftq.io.commit_ic_stall_cycles }
    }
  }

  fb.io.enq.valid := f4.io.deq.valid && !f4_delay
  fb.io.enq.bits  := f4.io.deq.bits
  fb.io.enq.bits.sfbs    := Mux(f4_sfb_valid, UIntToOH(f4_sfb_idx), 0.U(fetchWidth.W)).asBools
  fb.io.enq.bits.shadowed_mask := (
    Mux(f4_sfb_valid, f4_sfb_mask(fetchWidth-1,0), 0.U(fetchWidth.W)) |
    f4.io.deq.bits.shadowed_mask.asUInt
  ).asBools

  // Export fetch buffer enqueue info for event counters
  val fb_enq_fire = fb.io.enq.fire
  val fb_enq_cnt  = PopCount(fb.io.enq.bits.mask)
  io.cpu.fb_enq_valid := fb_enq_fire
  io.cpu.fb_enq_cnt   := fb_enq_cnt


  ftq.io.predecode_enq.valid          := f4.io.deq.valid && fb.io.enq.ready && !f4_delay
  ftq.io.predecode_enq.bits           := f4.io.deq.bits

  val bpd_update_arbiter = Module(new Arbiter(new BranchPredictionUpdate, 2))
  bpd_update_arbiter.io.in(0).valid := ftq.io.bpdupdate.valid
  bpd_update_arbiter.io.in(0).bits  := ftq.io.bpdupdate.bits
  assert(bpd_update_arbiter.io.in(0).ready)
  bpd_update_arbiter.io.in(1) <> f4_btb_corrections.io.deq
  bpd.io.update := ftq.io.bpdupdate
  bpd_update_arbiter.io.out.ready := true.B

  // 从 FTQ 来的 RAS 更新具有更高优先级 
  when (ftq.io.ras_update && enableRasTopRepair.B) {
    // FTQ 负责更新 ras top 的内容
    bpd.io.backend_ras_update_valid := true.B
    bpd.io.backend_ras_update_idx  := ftq.io.ras_update_idx
    bpd.io.backend_ras_update_addr  := ftq.io.ras_update_pc

    assert (RegNext(io.cpu.redirect_val), "RAS updates should only occur on next cycle of redirects" )
    assert (!f3.io.deq.valid, "f3 should be cleared by redirects when RAS updates occur")
  }


  // -------------------------------------------------------
  // **** To Core (F5) ****
  // -------------------------------------------------------

  io.cpu.fetchpacket <> fb.io.deq
  io.cpu.get_pc <> ftq.io.get_ftq_pc

  // TODO: 当没有发生 mispredict 时, ghist 
  // when (!io.cpu.mispredict_val) {
  //   ftq.io.get_ftq_pc(1).ftq_idx := s2_ftq_idx
  // }

  ftq.io.deq := io.cpu.commit
  ftq.io.brupdate := io.cpu.brupdate

  ftq.io.redirect.valid   := io.cpu.redirect_val
  ftq.io.redirect.bits    := io.cpu.redirect_ftq_idx
  ftq.io.redirect_target  := io.cpu.redirect_pc
  ftq.io.rob_flush        := io.cpu.rob_flush
  ftq.io.rob_flush_pc_lob := io.cpu.rob_flush_pc_lob
  fb.io.clear := false.B

  when (io.cpu.sfence.valid) {

  }.elsewhen (io.cpu.redirect_flush) {

    ftq.io.redirect.valid := io.cpu.redirect_val
    ftq.io.redirect.bits  := io.cpu.redirect_ftq_idx

    // 如果预测错误的 cfi 指令是 call 或者 ret 指令, 则
    // 新的 ras top 和 ftq update ras idx 不相同
    // 后端重定向更新 ras top 指针的内容
    bpd.io.backend_ras_top_update_valid := true.B
    bpd.io.backend_ras_top_update_idx  := io.cpu.redirect_ghist.ras_idx

    // 如果预测错误的 CFI 是 call 指令，在同一周期将 call 的返回地址写入 RAS
    // 这不与 FTQ 的 ras_update 冲突，因为 FTQ 的 ras_update 在下一周期才拉高
    when (io.cpu.redirect_is_call) {
      bpd.io.backend_ras_update_valid := true.B
      bpd.io.backend_ras_update_idx   := io.cpu.redirect_ghist.ras_idx
      bpd.io.backend_ras_update_addr  := io.cpu.redirect_call_addr
    }
  }

  ftq.io.debug_ftq_idx := io.cpu.debug_ftq_idx
  io.cpu.debug_fetch_pc := ftq.io.debug_fetch_pc

  //////////////////////////////////////
  // bpd s0 逻辑                       //
  //////////////////////////////////////

  // TODO: s0 和 s1 寄存器可以合并吗
  val predecode_suppress_f3_redirect = f3.io.deq.valid && bpd.io.resp.f3_ftq_idx === f3_fetch_bundle.ftq_idx
  bpd_f3_real_redirect := bpd.io.resp.f3_redirect && !predecode_suppress_f3_redirect

  val select_predecode  = predecode_redirect || (f3.io.deq.valid && bpd.io.resp.f3_pred_valid &&
                          f3_fetch_bundle.ftq_idx === bpd.io.resp.f3_ftq_idx)
  val merged_f3_next_pc      = Mux(select_predecode, f3_predicted_target, bpd.io.resp.f3_next_pc)
  val merged_f3_ftq_idx      = Mux(select_predecode, f3_fetch_bundle.ftq_idx, bpd.io.resp.f3_ftq_idx)
  val merged_f3_next_ftq_idx = merged_f3_ftq_idx + 1.U
  val merged_f3_next_ghist   = Mux(select_predecode, f3_predicted_ghist, bpd.io.resp.f3_next_ghist)
  val merged_f3_next_valid   = Mux(select_predecode, !(f3_fetch_bundle.xcpt_pf_if || f3_fetch_bundle.xcpt_ae_if),
                                  true.B)
  val merged_f3_redirect     = Mux(select_predecode, predecode_redirect, bpd.io.resp.f3_redirect) 
  // 当 select_predecode 为 true 时，预译码必为有效
  val merged_f3_valid        = Mux(select_predecode, true.B, bpd.io.resp.f3_pred_valid)

  ftq.io.merged_f3_target    := merged_f3_next_pc
  icache.io.bpd_f3_flush := merged_f3_redirect
  icache.io.bpd_f3_ftq_idx := merged_f3_ftq_idx
  icache.io.mshr_flush := predecode_redirect || merged_f3_redirect

  when (bpd.io.resp.f1_pred_valid) {
    s0_bpd_valid   := true.B
    s0_bpd_vpc     := bpd.io.resp.f1_next_pc
    s0_bpd_ftq_idx := bpd.io.resp.f1_ftq_idx + 1.U
    s0_bpd_ghist   := bpd.io.resp.f1_next_ghist
    s0_bpd_tsrc_debug    := BPD_S0_SRC_F1_DEBUG
  }
  when (bpd.io.resp.f2_redirect) {
    s0_bpd_valid   := true.B
    s0_bpd_vpc     := bpd.io.resp.f2_next_pc
    s0_bpd_ftq_idx := bpd.io.resp.f2_ftq_idx + 1.U
    s0_bpd_ghist   := bpd.io.resp.f2_next_ghist

    s0_bpd_tsrc_debug    := BPD_S0_SRC_F2_DEBUG

    bpd_f1_clear   := true.B
  }
  when (merged_f3_redirect) {
    s0_bpd_valid   := merged_f3_next_valid
    s0_bpd_vpc     := merged_f3_next_pc
    s0_bpd_ftq_idx := merged_f3_next_ftq_idx
    s0_bpd_ghist   := merged_f3_next_ghist

    s0_bpd_tsrc_debug    := Mux(select_predecode,
      BPD_S0_SRC_PREDECODE_DEBUG,
      BPD_S0_SRC_F3_DEBUG)

    bpd_f1_clear   := true.B
    bpd_f2_clear   := true.B
  }
  // TODO: 对于 sfence.vma，有必要 flush bpd 吗
  when (io.cpu.sfence.valid) {
    s0_bpd_valid   := false.B
    s0_bpd_ftq_idx := s0_bpd_ftq_idx_reg

    s0_bpd_tsrc_debug    := BPD_S0_SRC_SFENCE_DEBUG

    bpd_f1_clear   := true.B
    bpd_f2_clear   := true.B
  }.elsewhen (io.cpu.redirect_flush) {
    s0_bpd_valid   := io.cpu.redirect_val
    s0_bpd_vpc     := io.cpu.redirect_pc
    s0_bpd_ftq_idx := Mux(io.cpu.redirect_val, ftq.io.redirect_ftq_idx + 1.U, s0_bpd_ftq_idx_reg)
    s0_bpd_ghist   := io.cpu.redirect_ghist

    s0_bpd_tsrc_debug    := BPD_S0_SRC_REDIRECT_DEBUG
  
    bpd_f1_clear   := true.B
    bpd_f2_clear   := true.B
  }

  // 当 bpd s0 希望写入的 ftq full 时，暂停 bpd 流水线，使用
  // s0 寄存器暂存 bpd 预测请求
  val bpd_ahead_limit = WireDefault(false.B) 
  val bpd_to_ftq_not_ready = isFull(ftq.io.bpd_commit_ptr, s0_bpd_ftq_idx)
  s0_bpd_real_valid := s0_bpd_valid && !bpd_to_ftq_not_ready && !bpd_ahead_limit
  when (!s0_bpd_real_valid) {
    s0_bpd_valid_reg     := s0_bpd_valid
    s0_bpd_vpc_reg       := s0_bpd_vpc
    s0_bpd_ghist_reg     := s0_bpd_ghist
    s0_bpd_ftq_idx_reg   := s0_bpd_ftq_idx
    s0_bpd_tsrc_debug_reg      := s0_bpd_tsrc_debug
  } .otherwise {
    // 如果成功发射到 s1，则清空 valid 寄存器
    s0_bpd_valid_reg     := false.B
    s0_bpd_ftq_idx_reg   := s0_bpd_ftq_idx + 1.U
    s0_bpd_tsrc_debug_reg      := BPD_S0_SRC_REG_DEBUG
  }

  //////////////////////////////////////
  // prefetch s0 逻辑                  //
  //////////////////////////////////////


  val sfence_reg = RegInit(false.B)
  when (io.cpu.sfence.valid) {
    sfence_reg := true.B
  } .elsewhen(io.cpu.redirect_flush) {
    sfence_reg := false.B
  }
  val predecode_flush_reg = RegInit(false.B)
  when(io.cpu.redirect_flush) {
    predecode_flush_reg := false.B
  } .elsewhen (predecode_redirect) {
    predecode_flush_reg := f3_fetch_bundle.xcpt_pf_if || f3_fetch_bundle.xcpt_ae_if
  }

  ftq.io.pf_ftq_idx := s0_pf_ftq_idx_reg

  val f1_next_ftq_idx = bpd.io.resp.f1_ftq_idx + 1.U
  val f2_next_ftq_idx = bpd.io.resp.f2_ftq_idx + 1.U

  trans_queue.io.flush := predecode_redirect || io.cpu.redirect_flush || io.cpu.sfence.valid
  trans_queue.io.bpd_f3_flush := merged_f3_redirect
  trans_queue.io.f3_ftq_idx := merged_f3_ftq_idx
  trans_queue.io.predecode_redirect_debug := predecode_redirect

  val s1_pf_should_replay = s1_pf_valid && !s1_pf_can_go && !f1_pf_clear
  when (s1_pf_should_replay && s1_pf_ppc_valid) {
    s1_pf_replay := true.B
    s1_pf_replay_ppc := icache.io.s1_pf_ppc
    s1_pf_replay_exp := icache.io.s1_pf_ppc_exp
  } .elsewhen(s1_pf_valid && s1_pf_can_advance) { // s1 pf fire 到 s2 时重置 s1_pf_replay
    s1_pf_replay := false.B
  }
  when (f1_pf_clear) {
    s1_pf_replay := false.B
  }

  val s1_pf_clear_by_f2 = bpd.io.resp.f2_redirect && (s1_pf_ftq_idx > bpd.io.resp.f2_ftq_idx)
  val s1_pf_clear_by_f3 = merged_f3_redirect && (s1_pf_ftq_idx > merged_f3_ftq_idx)
  f1_pf_clear := s1_pf_clear_by_f2 || s1_pf_clear_by_f3 || io.cpu.sfence.valid || io.cpu.redirect_flush

  val pf_can_use_ftq_info = ftq.io.pf_pc.valid && !sfence_reg && !predecode_flush_reg
  val pf_last_is_f1_pred = s0_pf_ftq_idx_reg === f1_next_ftq_idx && bpd.io.resp.f1_pred_valid
  val pf_last_is_f2_pred = s0_pf_ftq_idx_reg === f2_next_ftq_idx && bpd.io.resp.f2_pred_valid
  val pf_last_is_f3_pred = s0_pf_ftq_idx_reg === merged_f3_next_ftq_idx && merged_f3_valid

  val s0_pf_redirect_by_f2 = s0_pf_ftq_idx_reg >= f2_next_ftq_idx && bpd.io.resp.f2_redirect
  val s0_pf_redirect_by_f3 = s0_pf_ftq_idx_reg >= merged_f3_next_ftq_idx && merged_f3_redirect

  val pf_can_use_f1_pred = pf_last_is_f1_pred
  val pf_can_use_f2_pred = s0_pf_redirect_by_f2 || pf_last_is_f2_pred
  val pf_can_use_f3_pred = s0_pf_redirect_by_f3 || pf_last_is_f3_pred

  when (pf_can_use_ftq_info) {
    s0_pf_valid     := true.B
    s0_pf_vpc       := ftq.io.pf_pc.bits
    s0_pf_ftq_idx   := s0_pf_ftq_idx_reg
  }
  when (pf_can_use_f1_pred) {
    s0_pf_valid     := true.B
    s0_pf_vpc       := bpd.io.resp.f1_next_pc
    s0_pf_ftq_idx   := f1_next_ftq_idx
  }
  when (pf_can_use_f2_pred) {
    s0_pf_valid     := true.B
    s0_pf_vpc       := bpd.io.resp.f2_next_pc
    s0_pf_ftq_idx   := f2_next_ftq_idx
  }
  when (pf_can_use_f3_pred) {
    s0_pf_valid     := merged_f3_next_valid
    s0_pf_vpc       := merged_f3_next_pc
    s0_pf_ftq_idx   := merged_f3_next_ftq_idx
  }
  when (s1_pf_should_replay) {
    s0_pf_valid     := true.B
    s0_pf_vpc       := s1_pf_vpc
    s0_pf_ftq_idx   := s1_pf_ftq_idx
  }
  when (io.cpu.sfence.valid) {
    s0_pf_valid     := false.B
    s0_pf_vpc       := io.cpu.sfence.bits.addr
    s0_pf_ftq_idx   := s0_pf_ftq_idx_reg
  } .elsewhen(io.cpu.redirect_flush) {
    s0_pf_valid     := io.cpu.redirect_val
    s0_pf_vpc       := io.cpu.redirect_pc
    s0_pf_ftq_idx   := Mux(io.cpu.redirect_val, ftq.io.redirect_ftq_idx + 1.U, s0_pf_ftq_idx_reg)
  }

  // 当 pf s0 希望写入的 ftq full 时，暂停 prefetch 流水线，使用
  // s0 寄存器暂存 pf 请求
  val pf_to_ftq_not_ready = isFull(ftq.io.bpd_commit_ptr, s0_pf_ftq_idx)
  val sync_decouple_pf_stall = disableSyncDecouple.B && s0_pf_valid && (s0_pf_ftq_idx === s0_bpd_ftq_idx)
  s0_pf_real_valid := s0_pf_valid && !pf_to_ftq_not_ready && !icache.io.s0_pf_blocked && !sync_decouple_pf_stall
  when (!s0_pf_real_valid) {
    s0_pf_valid_reg     := s0_pf_valid
    s0_pf_vpc_reg       := s0_pf_vpc
    s0_pf_ftq_idx_reg   := s0_pf_ftq_idx
  } .otherwise {
    // 如果成功发射到 s1，则清空 valid 寄存器
    s0_pf_valid_reg   := false.B
    s0_pf_ftq_idx_reg := s0_pf_ftq_idx + 1.U
  }


  //////////////////////////////////////
  // ifu s0 逻辑                       //
  //////////////////////////////////////
  
  // 向 ftq 请求新的 fetch pc
  ftq.io.ifu_fetch_ftq_idx := s0_ifu_ftq_idx_reg

  val can_use_ftq_info = ftq.io.ifu_fetch_pc.valid && !sfence_reg && !predecode_flush_reg
  val last_is_f1_pred = s0_ifu_ftq_idx_reg === f1_next_ftq_idx && bpd.io.resp.f1_pred_valid
  val last_is_f2_pred = s0_ifu_ftq_idx_reg === f2_next_ftq_idx && bpd.io.resp.f2_pred_valid
  val last_is_f3_pred = s0_ifu_ftq_idx_reg === merged_f3_next_ftq_idx && merged_f3_valid

  // 因为 f2_next_ftq_idx / f3_next_ftq_idx 可能超过 s0_ifu_ftq_idx_reg 一圈，
  // 但是 s0_ifu_ftq_idx_reg 至多比它们大 1 或者 2。使用 >= 来判断
  val s0_redirect_by_f2 = s0_ifu_ftq_idx_reg >= f2_next_ftq_idx && bpd.io.resp.f2_redirect
  val s0_redirect_by_f3 = s0_ifu_ftq_idx_reg >= merged_f3_next_ftq_idx && merged_f3_redirect

  val can_use_f1_pred = last_is_f1_pred
  val can_use_f2_pred = s0_redirect_by_f2 || last_is_f2_pred
  val can_use_f3_pred = s0_redirect_by_f3 || last_is_f3_pred
  
  // f3 redirect 和 s2 replay 可能同时发生，当 f3 是更早一级流水线时，f3 redirect 优先级更高
  val f3_suppress_s2_replay = s2_ftq_idx === merged_f3_next_ftq_idx && merged_f3_redirect

  // ifu f1 可能被 f2 预测, f3 预测, s2 重放, sfence, 后端重定向清空
  val s1_clear_by_f2 = bpd.io.resp.f2_redirect && (s1_ftq_idx > bpd.io.resp.f2_ftq_idx)
  val s1_clear_by_f3 = merged_f3_redirect && (s1_ftq_idx > merged_f3_ftq_idx)
  f1_clear := s1_clear_by_f2 || s1_clear_by_f3 || s2_replay_happen ||
              io.cpu.sfence.valid || io.cpu.redirect_flush

  // ifu f2 可能被 f3 预测, sfence, 后端重定向清空
  val s2_clear_by_f3 = merged_f3_redirect && (s2_ftq_idx > merged_f3_ftq_idx)
  f2_clear := s2_clear_by_f3 || io.cpu.sfence.valid || io.cpu.redirect_flush

  // ifu predecode 和 f4 可能被 sfence, 后端重定向清空
  f3_clear    := io.cpu.sfence.valid || io.cpu.redirect_flush
  f4_clear    := io.cpu.sfence.valid || io.cpu.redirect_flush
  fb.io.clear := io.cpu.sfence.valid || io.cpu.redirect_flush

  // 来源 1: 来自 ftq 的新 fetch 请求
  when (can_use_ftq_info) {
    s0_valid     := true.B
    s0_ifu_vpc   := ftq.io.ifu_fetch_pc.bits
    s0_ftq_idx   := s0_ifu_ftq_idx_reg
    s0_is_replay := false.B
    s0_ifu_tsrc  := BSRC_1
    s0_ifu_tsrc_debug := IFU_S0_SRC_FTQ_DEBUG
  }
  // 来源 2: 来自 bpd 的 f1 预测
  when (can_use_f1_pred) {
    s0_valid     := true.B
    s0_ifu_vpc   := bpd.io.resp.f1_next_pc
    s0_ftq_idx   := f1_next_ftq_idx
    s0_is_replay := false.B
    s0_ifu_tsrc  := BSRC_1
    s0_ifu_tsrc_debug := IFU_S0_SRC_F1_DEBUG
  }
  // 来源 3: 来自 bpd 的 f2 预测或重定向
  when (can_use_f2_pred) {
    s0_valid     := true.B
    s0_ifu_vpc   := bpd.io.resp.f2_next_pc
    s0_ftq_idx   := f2_next_ftq_idx
    s0_is_replay := false.B
    s0_ifu_tsrc  := BSRC_2
    s0_ifu_tsrc_debug := IFU_S0_SRC_F2_DEBUG
  }
  // 来源 4: 来自 bpd 的 f3 预测或重定向
  when (can_use_f3_pred) {
    s0_valid     := merged_f3_next_valid
    s0_ifu_vpc   := merged_f3_next_pc
    s0_ftq_idx   := merged_f3_next_ftq_idx
    s0_is_replay := false.B
    s0_ifu_tsrc  := BSRC_3
    s0_ifu_tsrc_debug := Mux(select_predecode,
      IFU_S0_SRC_PREDECODE_DEBUG,
      IFU_S0_SRC_F3_DEBUG)
  }
  // 来源 5: 来自 s2 重放
  when (s2_replay_happen && !f3_suppress_s2_replay) {
    // TODO: 设置为 true 就行了？因为 s3 预译码检测到异常会 flush 的
    s0_valid     := !f3_enq_fire
    s0_ifu_vpc   := s2_vpc
    s0_ftq_idx   := Mux(f3_enq_fire, s2_ftq_idx + 1.U, s2_ftq_idx)
    s0_is_replay := !s2_tlb_miss
    s0_is_real_replay := true.B
    s0_replay_is_ic_miss := !s2_tlb_miss && !icache.io.resp.valid
    s0_ifu_tsrc  := s2_ifu_tsrc
    s0_ifu_tsrc_debug := IFU_S0_SRC_S2_REPLAY_DEBUG
  }
  // 来源 7: 来自 sfence.vma flush 或后端重定向
  when (io.cpu.sfence.valid) {
    s0_valid     := false.B
    s0_ifu_vpc   := io.cpu.sfence.bits.addr
    s0_ftq_idx   := s0_ifu_ftq_idx_reg
    s0_is_replay := false.B
    s0_is_real_replay := false.B
    s0_replay_is_ic_miss := false.B

    s0_is_sfence := true.B
    s0_ifu_tsrc_debug := IFU_S0_SRC_SFENCE_DEBUG
  }.elsewhen (io.cpu.redirect_flush) {
    s0_valid     := io.cpu.redirect_val
    s0_ifu_vpc   := io.cpu.redirect_pc
    s0_ftq_idx   := Mux(io.cpu.redirect_val, ftq.io.redirect_ftq_idx + 1.U, s0_ifu_ftq_idx_reg)
    s0_is_replay := false.B
    s0_is_real_replay := false.B
    s0_replay_is_ic_miss := false.B
    s0_ifu_tsrc  := BSRC_C
    s0_ifu_tsrc_debug := IFU_S0_SRC_REDIRECT_DEBUG
  }

  // 当 ifu s0 希望写入的 ftq full 时，暂停前端流水线，使用
  // s0 寄存器暂存 fetch 请求
  val ifu_to_ftq_not_ready = isFull(ftq.io.bpd_commit_ptr, s0_ftq_idx)
  val sync_decouple_ifu_stall = disableSyncDecouple.B && s0_valid && (s0_ftq_idx === s0_bpd_ftq_idx)
  s0_ifu_real_valid := s0_valid && !ifu_to_ftq_not_ready && !sync_decouple_ifu_stall
  // Export s0 stall conditions for perf counters
  io.cpu.s0_ifu_real_not_valid := !s0_ifu_real_valid
  io.cpu.s0_ifu_ftq_backpress  := s0_valid && ifu_to_ftq_not_ready
  io.cpu.predecode_redirect_count := predecode_redirect
  when (!s0_ifu_real_valid) {
    s0_ifu_valid_reg   := s0_valid
    s0_ifu_vpc_reg     := s0_ifu_vpc
    s0_ifu_ftq_idx_reg := s0_ftq_idx
    s0_ifu_tsrc_reg    := s0_ifu_tsrc
    s0_ifu_tsrc_debug_reg := s0_ifu_tsrc_debug
  } .otherwise {
    // 如果成功发射到 s1，则清空 valid 寄存器
    s0_ifu_valid_reg   := false.B
    s0_ifu_ftq_idx_reg := s0_ftq_idx + 1.U
    s0_ifu_tsrc_debug_reg := IFU_S0_SRC_REG_DEBUG
  }

  // --------------------------------------------------------
  // **** Frontend bubble statistics by clear source ****
  // Bubble count is measured as the FTQ pointer distance between
  // the last S0 IFU FTQ index and the current S0 FTQ index.
  // This corresponds to the number of frontend bubbles created
  // by different clear/redirect sources.

  // distanceBetween returns a value in range [0, ftqSz]
  val s0_bubble = Wire(UInt((log2Ceil(ftqSz)+1).W))
  s0_bubble := distanceBetween(s0_ifu_ftq_idx_reg, s0_ftq_idx)

  // Default bubble contributions per source
  val f2_clear_bubble        = WireDefault(0.U(4.W))
  val f3_clear_bubble        = WireDefault(0.U(4.W))
  val predecode_clear_bubble = WireDefault(0.U(4.W))
  val rob_flush_bubble       = WireDefault(0.U((log2Ceil(ftqSz)+1).W))
  val mispred_flush_bubble   = WireDefault(0.U((log2Ceil(ftqSz)+1).W))

  when (s0_ifu_real_valid) {
    when (s0_ifu_tsrc_debug === IFU_S0_SRC_F2_DEBUG) {
      // Bubbles created by F2 clear
      f2_clear_bubble := s0_bubble(3,0)
    } .elsewhen (s0_ifu_tsrc_debug === IFU_S0_SRC_F3_DEBUG) {
      // Bubbles created by F3 clear
      f3_clear_bubble := s0_bubble(3,0)
    } .elsewhen (s0_ifu_tsrc_debug === IFU_S0_SRC_PREDECODE_DEBUG) {
      // Bubbles created by predecode clear
      predecode_clear_bubble := s0_bubble(3,0)
    } .elsewhen (s0_ifu_tsrc_debug === IFU_S0_SRC_REDIRECT_DEBUG) {
      // Backend-originated flushes (ROB flush vs branch mispredict)
      when (io.cpu.rob_flush) {
        rob_flush_bubble := s0_bubble
      } .otherwise {
        mispred_flush_bubble := s0_bubble
      }
    }
  }

  // Export bubble statistics to core for event counters
  io.cpu.f2_clear_bubble        := f2_clear_bubble
  io.cpu.f3_clear_bubble        := f3_clear_bubble
  io.cpu.predecode_clear_bubble := predecode_clear_bubble
  io.cpu.rob_flush_bubble       := rob_flush_bubble
  io.cpu.mispred_flush_bubble   := mispred_flush_bubble

  val bpd_ifu_dist = Wire(UInt((log2Ceil(ftqSz) + 1).W))
  bpd_ifu_dist := distanceBetween(s0_bpd_ftq_idx, s0_ftq_idx)

  // Export BPD-ahead-of-IFU distance bucket for perf counters (0-6)
  val dist_bucket = Wire(UInt(3.W))
  when (bpd_ifu_dist <= 1.U) {
    dist_bucket := 0.U
  } .elsewhen (bpd_ifu_dist <= 3.U) {
    dist_bucket := 1.U
  } .elsewhen (bpd_ifu_dist <= 5.U) {
    dist_bucket := 2.U
  } .elsewhen (bpd_ifu_dist <= 8.U) {
    dist_bucket := 3.U
  } .elsewhen (bpd_ifu_dist <= 12.U) {
    dist_bucket := 4.U
  } .elsewhen (bpd_ifu_dist <= 16.U) {
    dist_bucket := 5.U
  } .otherwise {
    dist_bucket := 6.U
  }
  io.cpu.bpd_ifu_dist_bucket := dist_bucket

  override def toString: String =
    (BoomCoreStringPrefix("====Overall Frontend Params====") + "\n"
    + icache.toString + bpd.toString)

  // Printf
  if (IN_SIMULATION) {
    // Add a free-running cycle counter for simulation
    val (cycleCount, _) = Counter(true.B, Int.MaxValue)

    val f4_with_delay = PlusArg("f4-with-delay", 0, "force f4 to have delay", 32)
    val delay_counter = RegInit(0.U(32.W))
    when (f4_with_delay =/= 0.U) {
      delay_counter := delay_counter + 1.U
      when (delay_counter === f4_with_delay - 1.U) {
        delay_counter := 0.U
      }
    }
    when (delay_counter =/= 0.U) {
      f4_delay := true.B
    }

    assert (s0_bpd_ftq_idx >= s0_ftq_idx || isFull(s0_bpd_ftq_idx, s0_ftq_idx),
            "BPD s0 ftq idx should never be behind IFU s0 ftq idx")
    val bpd_ahead_limit_number = PlusArg("bpd-ahead-limit", 63, "limit bpd ahead of ifu s0", 6)
    require(ftqSz < 60)  // 防止 bpd_ifu_dist 超过 bpd_ahead_limit_number 的上限

    when (bpd_ahead_limit_number < bpd_ifu_dist) {
      bpd_ahead_limit := true.B
    }

    val s0_bpd_printf = PlusArg("s0-bpd-printf", 0, "print s0 bpd state", 1)
    when (s0_bpd_printf(0)) {
      printf(p"[${cycleCount} S0 BPD] src=")
      when (s0_bpd_tsrc_debug === BPD_S0_SRC_REG_DEBUG) {
        printf(p"REG")
      } .elsewhen (s0_bpd_tsrc_debug === BPD_S0_SRC_F1_DEBUG) {
        printf(p"F1")
      } .elsewhen (s0_bpd_tsrc_debug === BPD_S0_SRC_F2_DEBUG) {
        printf(p"F2")
      } .elsewhen (s0_bpd_tsrc_debug === BPD_S0_SRC_PREDECODE_DEBUG) {
        printf(p"PREDECODE")
      } .elsewhen (s0_bpd_tsrc_debug === BPD_S0_SRC_F3_DEBUG) {
        printf(p"F3")
      } .elsewhen (s0_bpd_tsrc_debug === BPD_S0_SRC_SFENCE_DEBUG) {
        printf(p"SFENCE")
      } .elsewhen (s0_bpd_tsrc_debug === BPD_S0_SRC_REDIRECT_DEBUG) {
        printf(p"REDIRECT")
      } .otherwise {
        printf(p"UNKNOWN")
      }
      printf(p" vpc=${Hexadecimal(s0_bpd_vpc)} " +
        p"ftq_idx=${Hexadecimal(s0_bpd_ftq_idx.value)} " +
        p"valid=${s0_bpd_valid} " +
        p"real_valid=${s0_bpd_real_valid}\n")
    }

    val s0_ifu_printf = PlusArg("s0-ifu-printf", 0, "print s0 ifu state", 1)
    when (s0_ifu_printf(0)) {
      printf(p"[${cycleCount} S0 IFU] src=")
      when (s0_ifu_tsrc_debug === IFU_S0_SRC_REG_DEBUG) {
        printf(p"REG")
      } .elsewhen (s0_ifu_tsrc_debug === IFU_S0_SRC_FTQ_DEBUG) {
        printf(p"FTQ")
      } .elsewhen (s0_ifu_tsrc_debug === IFU_S0_SRC_F1_DEBUG) {
        printf(p"F1")
      } .elsewhen (s0_ifu_tsrc_debug === IFU_S0_SRC_F2_DEBUG) {
        printf(p"F2")
      } .elsewhen (s0_ifu_tsrc_debug === IFU_S0_SRC_PREDECODE_DEBUG) {
        printf(p"PREDECODE")
      } .elsewhen (s0_ifu_tsrc_debug === IFU_S0_SRC_F3_DEBUG) {
        printf(p"F3")
      } .elsewhen (s0_ifu_tsrc_debug === IFU_S0_SRC_S2_REPLAY_DEBUG) {
        printf(p"S2_REPLAY")
      } .elsewhen (s0_ifu_tsrc_debug === IFU_S0_SRC_SFENCE_DEBUG) {
        printf(p"SFENCE")
      } .elsewhen (s0_ifu_tsrc_debug === IFU_S0_SRC_REDIRECT_DEBUG) {
        printf(p"REDIRECT")
      } .otherwise {
        printf(p"UNKNOWN")
      }
      printf(p" vpc=${Hexadecimal(s0_ifu_vpc)} " +
        p"ftq_idx=${Hexadecimal(s0_ftq_idx.value)} " +
        p"valid=${s0_valid} " +
        p"real_valid=${s0_ifu_real_valid}\n")
    }

    val predecode_targte_printf = PlusArg("predecode-target-printf", 0, "print predecode targets", 1)
    when (predecode_targte_printf(0)) {
      when (f3.io.deq.fire) {
        val f3_fetch_pc = f3_aligned_pc + (PriorityEncoder(f3_imemresp.mask.asUInt) << 1)
        printf(p"[${cycleCount} F3] pc=${Hexadecimal(f3_fetch_pc)} " +
          p"mask=${Binary(f3_fetch_bundle.mask)} " +
          p"target=${Hexadecimal(f3_fetch_bundle.next_pc)} " +
          p"pred source=${f3_fetch_bundle.fsrc} " +
          p"taken=${Binary(f3_fetch_bundle.cfi_idx.valid)} " +
          p"cfi_idx=${f3_fetch_bundle.cfi_idx} " +
          p"cfi_is_call=${f3_fetch_bundle.cfi_is_call} " +
          p"cfi_is_ret=${f3_fetch_bundle.cfi_is_ret}\n")
        printf(p"f3 target0=${Hexadecimal(f3_bpd_resp.io.deq.bits.preds.jal_targets_debug(0))} " +
          p"target1=${Hexadecimal(f3_bpd_resp.io.deq.bits.preds.jal_targets_debug(1))} " +
          p"target2=${Hexadecimal(f3_bpd_resp.io.deq.bits.preds.jal_targets_debug(2))} " +
          p"target3=${Hexadecimal(f3_bpd_resp.io.deq.bits.preds.jal_targets_debug(3))}\n")
      }
    }
    val ftq_full_stall = PlusArg("ftq-full-stall-printf", 0, "print ftq full stalls", 1)
    val fb_insts_num = RegInit(0.U(32.W))
    val fb_enq_number = PopCount(fb.io.enq.bits.mask).asUInt
    val fb_deq_number = PopCount(io.cpu.dec_fire).asUInt
    // val fb_deq_number = Mux(fb.io.deq.fire, coreWidth.U, 0.U).asSInt
    when (fb.io.clear) {
      fb_insts_num := 0.U
    } .elsewhen (fb.io.enq.fire) {
      fb_insts_num := (fb_insts_num + fb_enq_number) - fb_deq_number
    } .otherwise {
      fb_insts_num := fb_insts_num - fb_deq_number
    }
    when (ftq_full_stall(0)) {
      when (!s0_ifu_real_valid && s0_valid) {
        printf(p"[${cycleCount} FTQ Full Stall] IFU pc=${Hexadecimal(s0_ifu_vpc)}, fb number=${fb_insts_num}\n")
      }
      when (!s0_bpd_real_valid && s0_bpd_valid) {
        printf(p"[${cycleCount} FTQ Full Stall] BPD pc=${Hexadecimal(s0_bpd_vpc)}, fb number=${fb_insts_num}\n")
      }
    }
    // assert (fb_insts_num >= 0.U, "FetchBuffer instruction count should never be negative")
    assert (fb_insts_num <= numFetchBufferEntries.U,
            "FetchBuffer instruction count should never exceed fetch buffer size")

    val cmd_ras_printf = PlusArg("ras-printf", 0, "print RAS updates", 1)
    when (cmd_ras_printf(0)) {
      when (ftq.io.ras_update) {
        val ras_update_ftq = RegNext(io.cpu.redirect_ftq_idx)
        printf(p"[${cycleCount} Backend RAS Update] ftq=${Hexadecimal(ras_update_ftq)} idx=${Hexadecimal(ftq.io.ras_update_idx)} " +
          p"addr=${Hexadecimal(ftq.io.ras_update_pc)}\n")
      } .elsewhen(bpd.io.predecode_ras_update_valid) {
        when (f3_fetch_bundle.cfi_is_call && f3_fetch_bundle.cfi_idx.valid) {
          val call_pc = bpd.io.predecode_ras_update_addr - Mux(f3_is_rvc(f3_fetch_bundle.cfi_idx.bits), 2.U, 4.U)
          when (predecode_redirect) {
            printf(p"[${cycleCount} Predecode Update RAS Push] ftq=${Hexadecimal(f3_fetch_bundle.ftq_idx.value)} " +
              p"idx=${Hexadecimal(bpd.io.predecode_ras_update_idx)} " +
              p"addr=${Hexadecimal(bpd.io.predecode_ras_update_addr)} pc=${Hexadecimal(call_pc)}\n")
          } .otherwise {
            printf(p"[${cycleCount} Predecode RAS Push] ftq=${Hexadecimal(f3_fetch_bundle.ftq_idx.value)} " +
              p"idx=${Hexadecimal(bpd.io.predecode_ras_update_idx)} " +
              p"addr=${Hexadecimal(bpd.io.predecode_ras_update_addr)} pc=${Hexadecimal(call_pc)}\n")
          }
        } .elsewhen (f3_fetch_bundle.cfi_is_ret && f3_fetch_bundle.cfi_idx.valid) {
          val ret_pc = f3_aligned_pc + (f3_fetch_bundle.cfi_idx.bits << 1) + Mux(
            f3_fetch_bundle.cfi_npc_plus4, 4.U, 2.U) - Mux(f3_is_rvc(f3_fetch_bundle.cfi_idx.bits), 2.U, 4.U)
          when (predecode_redirect) {
            printf(p"[${cycleCount} Predecode Update RAS Pop] ftq=${Hexadecimal(f3_fetch_bundle.ftq_idx.value)} " +
              p"idx=${Hexadecimal(bpd.io.predecode_ras_update_idx)} " +
              p"addr=${Hexadecimal(bpd.io.predecode_ras_update_addr)}, pc=${Hexadecimal(ret_pc)}\n")
          } .otherwise {
            printf(p"[${cycleCount} Predecode RAS Pop] ftq=${Hexadecimal(f3_fetch_bundle.ftq_idx.value)} " +
              p"idx=${Hexadecimal(bpd.io.predecode_ras_update_idx)} " +
              p"addr=${Hexadecimal(bpd.io.predecode_ras_update_addr)} pc=${Hexadecimal(ret_pc)}\n")
          }
        } .otherwise {
          when (predecode_redirect) {
            printf(p"[${cycleCount} Predecode RAS Update no call ret] ftq=${Hexadecimal(f3_fetch_bundle.ftq_idx.value)} " +
              p"idx=${Hexadecimal(bpd.io.predecode_ras_update_idx)} " +
              p"addr=${Hexadecimal(bpd.io.predecode_ras_update_addr)}\n")
          }// .otherwise {
           // printf(p"[${cycleCount} Predecode RAS Update] idx=${Hexadecimal(bpd.io.predecode_ras_update_idx)} " +
           //   p"addr=${Hexadecimal(bpd.io.predecode_ras_update_addr)}\n")
          // }
        }
      } .elsewhen(bpd.io.f3_cfi_is_call_debug) {
        printf(p"[${cycleCount} F3 BPD RAS Push] ftq=${Hexadecimal(bpd.io.resp.f3_ftq_idx.value)} " +
          p"idx=${Hexadecimal(bpd.io.f3_ras_update_idx_debug)} " +
          p"addr=${Hexadecimal(bpd.io.f3_ras_update_addr_debug)} pc=${Hexadecimal(bpd.io.f3_cfi_call_addr_debug)}\n")
      } .elsewhen(bpd.io.f3_cfi_is_ret_debug) {
        val top_idx = WrapInc(bpd.io.f3_ras_top_update_idx_debug, nRasEntries)
        printf(p"[${cycleCount} F3 BPD RAS Pop] ftq=${Hexadecimal(bpd.io.resp.f3_ftq_idx.value)} " +
          p"idx=${Hexadecimal(top_idx)} " +
          p"addr=${Hexadecimal(bpd.io.resp.f3_next_pc)} pc=${Hexadecimal(bpd.io.f3_cfi_ret_addr_debug)}\n")
      }
    }

    val tlb_trans_printf = PlusArg("tlb-trans-printf", 0, "print tlb translations", 1)
    when (tlb_trans_printf(0)) {
      when (trans_queue.io.enq.fire) {
        printf(p"[${cycleCount} tlb trans enqueue] ftq_idx=${Hexadecimal(trans_queue.io.enq_ftq_idx.value)} " +
          p"pc=${Hexadecimal(s1_pf_vpc)}\n")
      }
      when (trans_queue.io.deq.fire) {
        printf(p"[${cycleCount} tlb trans dequeue] ftq_idx=${Hexadecimal(s1_ftq_idx.value)} " +
          p"pc=${Hexadecimal(s1_vpc)} deq_ftq_idx=${Hexadecimal(trans_queue.io.deq_ftq_idx_debug.value)}\n")
      }
    }

    val tlb_force_miss_num = PlusArg("tlb-force-miss-num", 0, "number of forced tlb misses", 32)
    val tlb_miss_counter = RegInit(0.U(32.W))
    when (tlb_force_miss_num =/= 0.U) {
      tlb_miss_counter := tlb_miss_counter + 1.U
      when (tlb_miss_counter === tlb_force_miss_num - 1.U) {
        tlb_miss_counter := 0.U
      }
    }
    when (tlb_miss_counter =/= 0.U) {
      tlb_force_miss := true.B
    }
  }

  // Assertions
  when (f3.io.deq.fire) {
    assert (f3_bpd_resp.io.deq.fire, "BPD F3 response should fire when F3 fires" )
  }

  when (!s0_ifu_real_valid && s0_valid) {
    assert(!s0_is_replay, "ifu should not stall replay")
  }
  assert (!(can_use_f1_pred && can_use_ftq_info), "should not be able to use both BPD F1 and FTQ info" )
  assert (!(can_use_f2_pred && can_use_ftq_info), "should not be able to use both BPD F2 and FTQ info" )
  assert (!(can_use_f3_pred && can_use_ftq_info && !select_predecode),
          "should not be able to use both BPD F3 and FTQ info" )

  when (s2_clear_by_f3) {
    assert (!s2_replay_happen || f3_suppress_s2_replay,
      "s2 should not be cleared by both f3 redirect and s2 replay" )
  }
  // 断言 bpd 和 ifu 的相对进度
  when (bpd.io.resp.f1_pred_valid) {
    assert (bpd.io.resp.f1_ftq_idx + 1.U === s0_ifu_ftq_idx_reg ||
            bpd.io.resp.f1_ftq_idx >= s0_ifu_ftq_idx_reg)
  }
  when (bpd.io.resp.f2_pred_valid) {
    assert (bpd.io.resp.f2_ftq_idx + 1.U === s0_ifu_ftq_idx_reg ||
            bpd.io.resp.f2_ftq_idx + 2.U === s0_ifu_ftq_idx_reg ||
            bpd.io.resp.f2_ftq_idx >= s0_ifu_ftq_idx_reg)
  }
  when (bpd.io.resp.f3_pred_valid) {
    assert (bpd.io.resp.f3_ftq_idx + 1.U === s0_ifu_ftq_idx_reg ||
            bpd.io.resp.f3_ftq_idx + 2.U === s0_ifu_ftq_idx_reg ||
            bpd.io.resp.f3_ftq_idx + 3.U === s0_ifu_ftq_idx_reg ||
            bpd.io.resp.f3_ftq_idx >= s0_ifu_ftq_idx_reg)
  }
  when (s1_valid) {
    when (bpd.io.resp.f1_pred_valid) {
      assert (s1_ftq_idx <= bpd.io.resp.f1_ftq_idx,
              "s1 ftq idx can't go faster than bpd f1 ftq idx")
      assert (!isFull(s1_ftq_idx, bpd.io.resp.f1_ftq_idx),
              "bpd f1 ftq idx should not lap s1 ftq idx")
    }
    when (bpd.io.resp.f2_pred_valid) {
      assert (s1_ftq_idx <= bpd.io.resp.f2_ftq_idx || s1_ftq_idx === bpd.io.resp.f2_ftq_idx + 1.U,
        "s1 ftq idx can't go faster than bpd f2 ftq idx + 1")
      assert (!isFull(s1_ftq_idx, bpd.io.resp.f2_ftq_idx),
        "bpd f2 ftq idx should not lap s1 ftq idx")
    }
    when (bpd.io.resp.f3_pred_valid) {
      assert (s1_ftq_idx <= bpd.io.resp.f3_ftq_idx || s1_ftq_idx === bpd.io.resp.f3_ftq_idx + 1.U ||
              s1_ftq_idx === bpd.io.resp.f3_ftq_idx + 2.U,
        "s1 ftq idx can't go faster than bpd f3 ftq idx + 2")
      assert (!isFull(s1_ftq_idx, bpd.io.resp.f3_ftq_idx),
        "bpd f3 ftq idx should not lap s1 ftq idx")
    }
  }
  when (s2_valid) {
    when (bpd.io.resp.f2_pred_valid) {
      assert (s2_ftq_idx <= bpd.io.resp.f2_ftq_idx,
              "s2 ftq idx can't go faster than bpd f2 ftq idx")
      assert (!isFull(s2_ftq_idx, bpd.io.resp.f2_ftq_idx),
              "bpd f2 ftq idx should not lap s2 ftq idx")
    }
    when (bpd.io.resp.f3_pred_valid) {
      assert (s2_ftq_idx <= bpd.io.resp.f3_ftq_idx || s2_ftq_idx === bpd.io.resp.f3_ftq_idx + 1.U,
        "s2 ftq idx can't go faster than bpd f3 ftq idx + 1")
      assert (!isFull(s2_ftq_idx, bpd.io.resp.f3_ftq_idx),
        "bpd f3 ftq idx should not lap s2 ftq idx")
    }
  }
  when (f3.io.deq.valid) {
    when (bpd.io.resp.f3_pred_valid) {
      assert (f3_fetch_bundle.ftq_idx <= bpd.io.resp.f3_ftq_idx,
        "f3 ftq idx can't go faster than bpd f3 ftq idx")
      assert (!isFull(f3_fetch_bundle.ftq_idx, bpd.io.resp.f3_ftq_idx),
        "bpd f3 ftq idx should not lap f3 ftq idx")
    }
  }
  when (s1_valid) {
    assert (s1_ftq_idx < s0_ifu_ftq_idx_reg, 
      "s1 ftq idx can't go faster than ifu s0 ftq idx" )
  }
  when (s2_valid) {
    assert (s2_ftq_idx < s0_ifu_ftq_idx_reg, 
      "s2 ftq idx can't go faster than ifu s0 ftq idx" )
    assert (!s1_valid || s2_ftq_idx < s1_ftq_idx, 
      "s2 ftq idx must be less than s1 ftq idx" )
  }
  when (f3.io.deq.valid) {
    assert (f3_fetch_bundle.ftq_idx < s0_ifu_ftq_idx_reg, 
      "f3 ftq idx can't go faster than ifu s0 ftq idx" )
    assert (!s2_valid || f3_fetch_bundle.ftq_idx < s2_ftq_idx, 
      "f3 ftq idx must be less than s2 ftq idx" )
  }
  if (IN_SIMULATION) {
    when (trans_queue.io.deq.fire) {
      assert (trans_queue.io.deq_ftq_idx_debug === s1_ftq_idx,
        "transition queue dequeue ftq idx should match s1 ftq idx" )
    }
    when (s1_pf_should_replay) {
      assert (!s0_pf_redirect_by_f2 || pf_last_is_f2_pred)
      assert (!s0_pf_redirect_by_f3 || pf_last_is_f3_pred)
      assert (s0_pf_real_valid || icache.io.s0_pf_blocked,
        "pf can't replay only when icache s0 pf  is blocked" )
    }

    assert (s0_ifu_ftq_idx_reg >= s0_ftq_idx)
  }
  // TODO: 断言 bpd 给 ftq 的输入和 ftq 给 predecode 的输入一致 
}

package boom.v3.ifu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.{Field, Parameters}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util._

import boom.v3.common._
import boom.v3.util.{BoomCoreStringPrefix, WrapInc, WrapDec}




// A branch prediction for a single instruction
class BranchPrediction(implicit p: Parameters) extends BoomBundle()(p)
{
  // If this is a branch, do we take it?
  val taken           = Bool()

  // Is this a branch?
  val is_br           = Bool()
  // Is this a JAL?
  val is_jal          = Bool()
  // Is this a CALL?
  val is_call         = Bool()
  // Is this a RET?
  val is_ret          = Bool()
  // Does the next PC equal PC + 2?
  val npc_plus2       = Bool()
  // What is the target of his branch/jump? Do we know the target?
  val predicted_pc    = Valid(UInt(vaddrBitsExtended.W))
  // ras 的栈顶值，没有 valid 信号
  val ras_top         = UInt(vaddrBitsExtended.W)
}

// A branch prediction for a entire fetch-width worth of instructions
// This is typically merged from individual predictions from the banked
// predictor
class BranchPredictionBundle(implicit p: Parameters) extends BoomBundle()(p)
  with HasBoomFrontendParameters
{
  val pc = UInt(vaddrBitsExtended.W)
  val preds = Vec(fetchWidth, new BranchPrediction)
  val meta = Output(Vec(nBanks, UInt(bpdMaxMetaLength.W)))
  val lhist = Output(Vec(nBanks, UInt(localHistoryLength.W)))
}

class FetchPacketPredsInfo(implicit p: Parameters) extends BoomBundle()(p)
  with HasBoomFrontendParameters
{
  val br_taken = UInt(fetchWidth.W)
  // 第一个无条件跳转指令的目标地址
  val jal_target = UInt(vaddrBitsExtended.W)
  val jal_targets_debug = Vec(fetchWidth, UInt(vaddrBitsExtended.W))
  val ras_top = UInt(vaddrBitsExtended.W)
  // ras top 的 idx，用于修正 ras
  val ras_idx = UInt(log2Ceil(nRasEntries).W)
  // btb 是否命中
  val btb_hits = UInt(fetchWidth.W)
}

class FetchPacketPredTarget(implicit p: Parameters) extends BoomBundle()(p)
  with HasBoomFrontendParameters
{
  val f2_pred_target            = UInt(vaddrBitsExtended.W)
  val f2_pred_ghist_update_type = UInt(GHR_UPDATE_SZ.W)
  val f3_pred_target            = UInt(vaddrBitsExtended.W)
  val f3_pred_ghist_update_type = UInt(GHR_UPDATE_SZ.W)
}

class FetchPacketMetaInfo(implicit p: Parameters) extends BoomBundle()(p)
  with HasBoomFrontendParameters
{
  val pc = UInt(vaddrBitsExtended.W)
  val meta = Vec(nBanks, UInt(bpdMaxMetaLength.W))
  val ghist = new GlobalHistory
  val tsrc = UInt(BSRC_SZ.W)
  val fsrc = UInt(BSRC_SZ.W)
}

class F3PredsToFTQ(implicit p: Parameters) extends BoomBundle()(p)
  with HasBoomFrontendParameters
{
  val fetch_info  = new FetchPacketMetaInfo
  val preds_info  = new FetchPacketPredsInfo
  val target_info = new FetchPacketPredTarget
}


class BranchPredBundleWithGHist(implicit p: Parameters) extends BoomBundle()(p)
  with HasBoomFrontendParameters
{
  // 应该存放在 FTQ 中，不需要传递到 predecode 的信息
  val pc = UInt(vaddrBitsExtended.W)
  val meta = Output(Vec(nBanks, UInt(bpdMaxMetaLength.W)))
  // 分支预测器提供的预测信息，可能来自 f3 预测或 ftq
  val preds = new FetchPacketPredsInfo
  val tsrc = UInt(BSRC_SZ.W)
  val fsrc = UInt(BSRC_SZ.W)
  // 用于 predecode 校验的旧预测目标，可能来自 f2 预测或 ftq
  val check_target = UInt(vaddrBitsExtended.W)
  // 用于禁用 ret/jalr post-fetch correction 时信任的 BPD 目标
  val bpd_target = UInt(vaddrBitsExtended.W)
  val ghist_update_type = UInt(GHR_UPDATE_SZ.W)
  // 取该 fetch packet 时的信息，来自 ftq
  val ghist = new GlobalHistory
}


// A branch update for a fetch-width worth of instructions
class BranchPredictionUpdate(implicit p: Parameters) extends BoomBundle()(p)
  with HasBoomFrontendParameters
{
  // Indicates that this update is due to a speculated misprediction
  // Local predictors typically update themselves with speculative info
  // Global predictors only care about non-speculative updates
  val is_mispredict_update = Bool()
  val is_repair_update = Bool()
  val btb_mispredicts = UInt(fetchWidth.W)
  def is_btb_mispredict_update = btb_mispredicts =/= 0.U
  def is_commit_update = !(is_mispredict_update || is_repair_update)

  val pc            = UInt(vaddrBitsExtended.W)
  // Mask of instructions which are branches.
  // If these are not cfi_idx, then they were predicted not taken
  val br_mask       = UInt(fetchWidth.W)
  // Which CFI was taken/mispredicted (if any)
  val cfi_idx       = Valid(UInt(log2Ceil(fetchWidth).W))
  // Was the cfi taken?
  val cfi_taken     = Bool()
  // Was the cfi mispredicted from the original prediction?
  val cfi_mispredicted = Bool()
  // Was the cfi a br?
  val cfi_is_br     = Bool()
  // Was the cfi a jal/jalr?
  val cfi_is_jal  = Bool()
  // Was the cfi a jalr
  val cfi_is_jalr = Bool()
  // Was the cfi a call/ret?
  val cfi_is_call      = Bool()
  val cfi_is_ret       = Bool()
  val cfi_npc_plus4    = Bool()

  val ghist = new GlobalHistory
  val lhist = Vec(nBanks, UInt(localHistoryLength.W))


  // What did this CFI jump to?
  val target        = UInt(vaddrBitsExtended.W)

  val meta          = Vec(nBanks, UInt(bpdMaxMetaLength.W))
}

// A branch update to a single bank
class BranchPredictionBankUpdate(implicit p: Parameters) extends BoomBundle()(p)
  with HasBoomFrontendParameters
{
  val is_mispredict_update     = Bool()
  val is_repair_update         = Bool()

  val btb_mispredicts  = UInt(bankWidth.W)
  def is_btb_mispredict_update = btb_mispredicts =/= 0.U

  def is_commit_update = !(is_mispredict_update || is_repair_update)

  val pc               = UInt(vaddrBitsExtended.W)

  val br_mask          = UInt(bankWidth.W)
  val cfi_idx          = Valid(UInt(log2Ceil(bankWidth).W))
  val cfi_taken        = Bool()
  val cfi_mispredicted = Bool()

  val cfi_is_br        = Bool()
  val cfi_is_jal       = Bool()
  val cfi_is_jalr      = Bool()
  val cfi_is_call      = Bool()
  val cfi_is_ret       = Bool()
  val cfi_npc_plus4    = Bool()

  val ghist            = UInt(globalHistoryLength.W)
  val lhist            = UInt(localHistoryLength.W)

  val target           = UInt(vaddrBitsExtended.W)

  val meta             = UInt(bpdMaxMetaLength.W)
}

class BranchPredictionRequest(implicit p: Parameters) extends BoomBundle()(p)
{
  val ftq_idx = new FTQPtr
  val pc    = UInt(vaddrBitsExtended.W)
  val ghist = new GlobalHistory
  val tsrc = UInt(BSRC_SZ.W)
}


class BranchPredictionBankResponse(implicit p: Parameters) extends BoomBundle()(p)
  with HasBoomFrontendParameters
{
  val f1 = Vec(bankWidth, new BranchPrediction)
  val f2 = Vec(bankWidth, new BranchPrediction)
  val f3 = Vec(bankWidth, new BranchPrediction)
}

abstract class BranchPredictorBank(implicit p: Parameters) extends BoomModule()(p)
  with HasBoomFrontendParameters
{
  val metaSz = 0
  def nInputs = 1

  val mems: Seq[Tuple3[String, Int, Int]]

  val io = IO(new Bundle {
    val f0_valid = Input(Bool())
    val f0_pc    = Input(UInt(vaddrBitsExtended.W))
    val f0_mask  = Input(UInt(bankWidth.W))
    // Local history not available until end of f1
    val f1_ghist = Input(UInt(globalHistoryLength.W))
    val f1_lhist = Input(UInt(localHistoryLength.W))

    val resp_in = Input(Vec(nInputs, new BranchPredictionBankResponse))
    val resp = Output(new BranchPredictionBankResponse)

    // Store the meta as a UInt, use width inference to figure out the shape
    val f3_meta = Output(UInt(bpdMaxMetaLength.W))

    val f3_fire = Input(Bool())

    val update = Input(Valid(new BranchPredictionBankUpdate))

    // For RAS
    val f2_read_idx   = Input(UInt(log2Ceil(nRasEntries).W))

    val f3_write_valid = Input(Bool())
    val f3_write_idx   = Input(UInt(log2Ceil(nRasEntries).W))
    val f3_write_addr  = Input(UInt(vaddrBitsExtended.W))
  })
  io.resp := io.resp_in(0)

  io.f3_meta := 0.U

  val s0_idx       = fetchIdx(io.f0_pc)
  val s1_idx       = RegNext(s0_idx)
  val s2_idx       = RegNext(s1_idx)
  val s3_idx       = RegNext(s2_idx)

  val s0_valid = io.f0_valid
  val s1_valid = RegNext(s0_valid)
  val s2_valid = RegNext(s1_valid)
  val s3_valid = RegNext(s2_valid)

  val s0_mask = io.f0_mask
  val s1_mask = RegNext(s0_mask)
  val s2_mask = RegNext(s1_mask)
  val s3_mask = RegNext(s2_mask)

  val s0_pc = io.f0_pc
  val s1_pc = RegNext(s0_pc)

  val s0_update     = io.update
  val s0_update_idx = fetchIdx(io.update.bits.pc)
  val s0_update_valid = io.update.valid

  val s1_update     = RegNext(s0_update)
  val s1_update_idx = RegNext(s0_update_idx)
  val s1_update_valid = RegNext(s0_update_valid)



}



class BranchPredictor(implicit p: Parameters) extends BoomModule()(p)
 with HasBoomFrontendParameters
{
  val io = IO(new Bundle {

    // Requests and responses
    val f0_req = Input(Valid(new BranchPredictionRequest))

    val resp = Output(new Bundle {
      // 外部应该只关心 f3 的 prediction bundle
      // 因为只有 f3 的 meta, preds, ghist 会被写入到 FTQ 中
      val f3_meta = new FetchPacketMetaInfo
      val f3_preds_info = new FetchPacketPredsInfo
      val f3_preds_target = new FetchPacketPredTarget

      // prediction valid
      // TODO：是否要考虑 clear 信号呢？目前是没考虑的
      val f1_pred_valid = Bool()
      val f2_pred_valid = Bool()
      val f3_pred_valid = Bool()

      // predicted next ghist
      val f1_next_ghist = new GlobalHistory
      val f2_next_ghist = new GlobalHistory
      val f3_next_ghist = new GlobalHistory

      // predicted next pc
      val f1_next_pc = UInt(vaddrBitsExtended.W)
      val f2_next_pc = UInt(vaddrBitsExtended.W)
      val f3_next_pc = UInt(vaddrBitsExtended.W)

      val f1_ftq_idx = new FTQPtr
      val f2_ftq_idx = new FTQPtr
      val f3_ftq_idx = new FTQPtr

      // 表示 f2 或 f3 是否重定向之前的预测
      val f2_redirect = Bool()
      val f3_redirect = Bool()

    })

    // 用于 clear f1 和 f2 流水线，后续引入
    // FDIP 之后应该只需要一个单独的 flush 信号
    // TODO:是不是不需要 clear？
    val f1_clear = Input(Bool())
    val f2_clear = Input(Bool())

    val f3_fire = Input(Bool())

    // Update
    val update = Input(Valid(new BranchPredictionUpdate))

    // 预译码修改 ras 的表项内容
    val predecode_ras_update_valid = Input(Bool())
    val predecode_ras_update_idx   = Input(UInt(log2Ceil(nRasEntries).W))
    val predecode_ras_update_addr  = Input(UInt(vaddrBitsExtended.W))
    // 预译码修改 ras top 位置
    val predecode_ras_top_update_valid = Input(Bool())
    val predecode_ras_top_update_idx   = Input(UInt(log2Ceil(nRasEntries).W)) 
    // 后端重定向修正 ras 表项内容
    val backend_ras_update_valid = Input(Bool())
    val backend_ras_update_idx   = Input(UInt(log2Ceil(nRasEntries).W))
    val backend_ras_update_addr  = Input(UInt(vaddrBitsExtended.W))    
    // 后端重定向修正 ras top 位置
    val backend_ras_top_update_valid = Input(Bool())
    val backend_ras_top_update_idx   = Input(UInt(log2Ceil(nRasEntries).W))

    // ras debug
    val f3_cfi_is_call_debug = Output(Bool())
    val f3_cfi_call_addr_debug = Output(UInt(vaddrBitsExtended.W))
    val f3_cfi_is_ret_debug = Output(Bool())
    val f3_cfi_ret_addr_debug = Output(UInt(vaddrBitsExtended.W))
    val f3_ras_top_update_valid_debug = Output(Bool())
    val f3_ras_top_update_idx_debug   = Output(UInt(log2Ceil(nRasEntries).W))
    val f3_ras_update_valid_debug = Output(Bool())
    val f3_ras_update_idx_debug   = Output(UInt(log2Ceil(nRasEntries).W))
    val f3_ras_update_addr_debug  = Output(UInt(vaddrBitsExtended.W))
  })

  var total_memsize = 0
  val bpdStr = new StringBuilder
  bpdStr.append(BoomCoreStringPrefix("==Branch Predictor Memory Sizes==\n"))
  val banked_predictors = (0 until nBanks) map ( b => {
    val m = Module(if (useBPD) new ComposedBranchPredictorBank else new NullBranchPredictorBank)
    for ((n, d, w) <- m.mems) {
      bpdStr.append(BoomCoreStringPrefix(f"bank$b $n: $d x $w = ${d * w / 8}"))
      total_memsize = total_memsize + d * w / 8
    }
    m
  })
  bpdStr.append(BoomCoreStringPrefix(f"Total bpd size: ${total_memsize / 1024} KB\n"))
  override def toString: String = bpdStr.toString

  val banked_lhist_providers = Seq.fill(nBanks) { Module(if (localHistoryNSets > 0) new LocalBranchPredictorBank else new NullLocalBranchPredictorBank) }

  // RAS 相关
  val f3_ras_top_update_valid = WireDefault(false.B)
  val f3_ras_top_update_idx = Wire(UInt(log2Ceil(nRasEntries).W))
  val f3_ras_update_valid = WireDefault(false.B)
  val f3_ras_update_idx = Wire(UInt(log2Ceil(nRasEntries).W))
  val f3_ras_update_addr = Wire(UInt(vaddrBitsExtended.W))

  f3_ras_top_update_idx := DontCare
  f3_ras_update_idx := DontCare
  f3_ras_update_addr := DontCare

  io.f3_ras_top_update_valid_debug := f3_ras_top_update_valid
  io.f3_ras_top_update_idx_debug   := f3_ras_top_update_idx
  io.f3_ras_update_valid_debug := f3_ras_update_valid
  io.f3_ras_update_idx_debug   := f3_ras_update_idx
  io.f3_ras_update_addr_debug  := f3_ras_update_addr
  io.f3_cfi_is_call_debug := false.B
  io.f3_cfi_call_addr_debug := DontCare
  io.f3_cfi_is_ret_debug := false.B
  io.f3_cfi_ret_addr_debug := DontCare


  // 会不会改为 f3_ras_top_idx 更合适一些？
  val f2_ras_top_idx = RegInit(0.U(log2Ceil(nRasEntries).W))
  val f2_ras_top_idx_write = WireDefault(f2_ras_top_idx)
  f2_ras_top_idx := f2_ras_top_idx_write
  
  // 修改 ras top 的最高优先级是后端重定向
  when (io.backend_ras_top_update_valid) {
    f2_ras_top_idx_write := io.backend_ras_top_update_idx
  // 其次是预译码修改
  } .elsewhen (io.predecode_ras_top_update_valid) {
    f2_ras_top_idx_write := io.predecode_ras_top_update_idx
  // 最后是 f3 预测修改
  } .elsewhen (f3_ras_top_update_valid) {
    f2_ras_top_idx_write := f3_ras_top_update_idx
  }

  val ras_write_valid = WireDefault(false.B)
  val ras_write_idx   = Wire(UInt(log2Ceil(nRasEntries).W))
  ras_write_idx := DontCare
  val ras_write_addr  = Wire(UInt(vaddrBitsExtended.W))
  ras_write_addr := DontCare
  // 修改 ras 表项内容的最高优先级是后端重定向
  when (io.backend_ras_update_valid) {
    ras_write_valid := true.B
    ras_write_idx   := io.backend_ras_update_idx
    ras_write_addr  := io.backend_ras_update_addr
  // 其次是预译码修改
  } .elsewhen (io.predecode_ras_update_valid) {
    ras_write_valid := true.B
    ras_write_idx   := io.predecode_ras_update_idx
    ras_write_addr  := io.predecode_ras_update_addr
  } .elsewhen (f3_ras_update_valid) {
    ras_write_valid := true.B
    ras_write_idx   := f3_ras_update_idx
    ras_write_addr  := f3_ras_update_addr
  }

  if (nBanks == 1) {
    banked_lhist_providers(0).io.f0_valid := io.f0_req.valid
    banked_lhist_providers(0).io.f0_pc    := bankAlign(io.f0_req.bits.pc)

    banked_predictors(0).io.f0_valid := io.f0_req.valid
    banked_predictors(0).io.f0_pc    := bankAlign(io.f0_req.bits.pc)
    banked_predictors(0).io.f0_mask  := fetchMask(io.f0_req.bits.pc)

    banked_predictors(0).io.f1_ghist := RegNext(io.f0_req.bits.ghist.histories(0))
    banked_predictors(0).io.f1_lhist := banked_lhist_providers(0).io.f1_lhist

    banked_predictors(0).io.resp_in(0)           := (0.U).asTypeOf(new BranchPredictionBankResponse)
    // For RAS
    banked_predictors(0).io.f2_read_idx := f2_ras_top_idx_write
    banked_predictors(0).io.f3_write_valid := ras_write_valid
    banked_predictors(0).io.f3_write_idx := ras_write_idx
    banked_predictors(0).io.f3_write_addr := ras_write_addr
  } else {
    require(nBanks == 2)

    banked_predictors(0).io.resp_in(0)           := (0.U).asTypeOf(new BranchPredictionBankResponse)
    banked_predictors(1).io.resp_in(0)           := (0.U).asTypeOf(new BranchPredictionBankResponse)

    banked_predictors(0).io.f1_lhist := banked_lhist_providers(0).io.f1_lhist
    banked_predictors(1).io.f1_lhist := banked_lhist_providers(1).io.f1_lhist

    when (bank(io.f0_req.bits.pc) === 0.U) {
      banked_lhist_providers(0).io.f0_valid := io.f0_req.valid
      banked_lhist_providers(0).io.f0_pc    := bankAlign(io.f0_req.bits.pc)

      banked_lhist_providers(1).io.f0_valid := io.f0_req.valid
      banked_lhist_providers(1).io.f0_pc    := nextBank(io.f0_req.bits.pc)

      banked_predictors(0).io.f0_valid := io.f0_req.valid
      banked_predictors(0).io.f0_pc    := bankAlign(io.f0_req.bits.pc)
      banked_predictors(0).io.f0_mask  := fetchMask(io.f0_req.bits.pc)

      banked_predictors(1).io.f0_valid := io.f0_req.valid
      banked_predictors(1).io.f0_pc    := nextBank(io.f0_req.bits.pc)
      banked_predictors(1).io.f0_mask  := ~(0.U(bankWidth.W))
    } .otherwise {
      banked_lhist_providers(0).io.f0_valid := io.f0_req.valid && !mayNotBeDualBanked(io.f0_req.bits.pc)
      banked_lhist_providers(0).io.f0_pc    := nextBank(io.f0_req.bits.pc)

      banked_lhist_providers(1).io.f0_valid := io.f0_req.valid
      banked_lhist_providers(1).io.f0_pc    := bankAlign(io.f0_req.bits.pc)

      banked_predictors(0).io.f0_valid := io.f0_req.valid && !mayNotBeDualBanked(io.f0_req.bits.pc)
      banked_predictors(0).io.f0_pc    := nextBank(io.f0_req.bits.pc)
      banked_predictors(0).io.f0_mask  := ~(0.U(bankWidth.W))

      banked_predictors(1).io.f0_valid := io.f0_req.valid
      banked_predictors(1).io.f0_pc    := bankAlign(io.f0_req.bits.pc)
      banked_predictors(1).io.f0_mask  := fetchMask(io.f0_req.bits.pc)
    }
    when (RegNext(bank(io.f0_req.bits.pc) === 0.U)) {
      banked_predictors(0).io.f1_ghist  := RegNext(io.f0_req.bits.ghist.histories(0))
      banked_predictors(1).io.f1_ghist  := RegNext(io.f0_req.bits.ghist.histories(1))
    } .otherwise {
      banked_predictors(0).io.f1_ghist  := RegNext(io.f0_req.bits.ghist.histories(1))
      banked_predictors(1).io.f1_ghist  := RegNext(io.f0_req.bits.ghist.histories(0))
    }
  }


  for (i <- 0 until nBanks) {
    banked_lhist_providers(i).io.f3_taken_br := banked_predictors(i).io.resp.f3.map ( p =>
      p.is_br && p.predicted_pc.valid && p.taken
    ).reduce(_||_)
  }

  if (nBanks == 1) {
    val f1_preds = banked_predictors(0).io.resp.f1
    val f2_preds = banked_predictors(0).io.resp.f2
    val f3_preds = banked_predictors(0).io.resp.f3

    banked_predictors(0).io.f3_fire := io.f3_fire
    banked_lhist_providers(0).io.f3_fire := io.f3_fire

    // 重定向相关逻辑
    val s1_ftq_idx = RegNext(io.f0_req.bits.ftq_idx)
    val s2_ftq_idx = RegNext(s1_ftq_idx)
    val s3_ftq_idx = RegNext(s2_ftq_idx)

    val s1_valid = RegNext(io.f0_req.valid, false.B)
    val s2_valid = RegNext(s1_valid && !io.f1_clear, false.B)
    val s3_valid = RegNext(s2_valid && !io.f2_clear, false.B)

    val s1_vpc = RegNext(io.f0_req.bits.pc)
    val s2_vpc = RegNext(s1_vpc)
    val s3_vpc = RegNext(s2_vpc)

    val s1_ghist = RegNext(io.f0_req.bits.ghist)
    val s2_ghist = RegNext(s1_ghist)
    // 用于 RAS 的访问
    val s3_ghist_write = WireDefault(s2_ghist)
    val s3_ghist = RegNext(s3_ghist_write) 

    val s1_tsrc = RegNext(io.f0_req.bits.tsrc)
    val s2_tsrc = RegNext(s1_tsrc)
    val s3_tsrc = RegNext(s2_tsrc)

    val s3_fsrc = Reg(UInt(BSRC_SZ.W))
    // 默认是 f1 预测
    s3_fsrc := BSRC_1

    // 根据 f1 预测计算新的 ghist 和 next pc 
    val f1_mask = fetchMask(s1_vpc)
    val f1_redirects = (0 until fetchWidth) map { i =>
      f1_mask(i) && f1_preds(i).predicted_pc.valid &&
      (f1_preds(i).is_jal || (f1_preds(i).is_br && f1_preds(i).taken))
    }

    val f1_redirect_idx = PriorityEncoder(f1_redirects)
    val f1_do_redirect = f1_redirects.reduce(_||_) && useBPD.B
    val f1_targs = f1_preds.map(_.predicted_pc.bits)
    val f1_predicted_target = Mux(f1_do_redirect,
                                  f1_targs(f1_redirect_idx),
                                  nextFetch(s1_vpc))

    val (f1_predicted_ghist, f1_pred_ghist_update_type) = s1_ghist.update(
      f1_preds.map(p => p.is_br && p.predicted_pc.valid).asUInt & f1_mask,
      f1_preds(f1_redirect_idx).taken && f1_do_redirect,
      f1_preds(f1_redirect_idx).is_br,
      f1_redirect_idx,
      f1_do_redirect,
      s1_vpc,
      false.B,
      false.B)
    
    // 输出 f1 的预测
    io.resp.f1_pred_valid := s1_valid
    io.resp.f1_next_ghist := f1_predicted_ghist
    io.resp.f1_next_pc := f1_predicted_target
    io.resp.f1_ftq_idx := s1_ftq_idx

    // 根据 f2 预测计算新的 ghist 和 next pc
    val f2_last_pred_target = RegNext(f1_predicted_target)
    val f2_ghist_update_type = RegNext(f1_pred_ghist_update_type)
    val f2_mask = fetchMask(s2_vpc)
    val f2_redirects = (0 until fetchWidth) map { i =>
      f2_mask(i) && f2_preds(i).predicted_pc.valid &&
      (f2_preds(i).is_jal || (f2_preds(i).is_br && f2_preds(i).taken))
    }
    val f2_redirect_idx = PriorityEncoder(f2_redirects)
    val f2_targs = f2_preds.map(_.predicted_pc.bits)
    val f2_do_redirect = f2_redirects.reduce(_||_) && useBPD.B
    val f2_predicted_target = Mux(f2_do_redirect,
                                  f2_targs(f2_redirect_idx),
                                  nextFetch(s2_vpc))
    val (f2_predicted_ghist, f2_pred_ghist_update_type) = s2_ghist.update(
      f2_preds.map(p => p.is_br && p.predicted_pc.valid).asUInt & f2_mask,
      f2_preds(f2_redirect_idx).taken && f2_do_redirect,
      f2_preds(f2_redirect_idx).is_br,
      f2_redirect_idx,
      f2_do_redirect,
      s2_vpc,
      false.B,
      false.B)

    // 输出 f2 的预测
    io.resp.f2_pred_valid := s2_valid
    io.resp.f2_next_ghist := f2_predicted_ghist
    io.resp.f2_next_pc := f2_predicted_target

    // f2 重定向 f1
    require(NO_SHIFT_CONST == 0)
    require(SHIFT_ZERO_CONST == 1)
    val s2_ghist_all_zero = s2_ghist === (0.U).asTypeOf(new GlobalHistory)
    val shift_zero_or_no_shift = f2_pred_ghist_update_type(1) === 0.U && f2_ghist_update_type(1) === 0.U
    val f2_correct_ghist = !(s2_ghist_all_zero && shift_zero_or_no_shift) &&
                              f2_pred_ghist_update_type =/= f2_ghist_update_type && enableGHistStallRepair.B
    val f2_do_redirection = f2_correct_ghist || f2_predicted_target =/= f2_last_pred_target

    io.resp.f2_ftq_idx := s2_ftq_idx
    io.resp.f2_redirect := false.B
    when (s2_valid && f2_do_redirection) {
      io.resp.f2_redirect := true.B
      s3_fsrc := BSRC_2
    }

    // 根据 f3 预测, 结合预译码信息计算新的 ghist 和 next pc
    val f3_last_pred_target = RegNext(f2_predicted_target)
    val f3_ghist_update_type = RegNext(f2_pred_ghist_update_type)
    val f3_mask = fetchMask(s3_vpc)
    val f3_is_call = f3_preds.map(p => p.is_call && p.predicted_pc.valid)
    val f3_is_ret = f3_preds.map(p => p.is_ret && p.predicted_pc.valid)
    val f3_redirects = (0 until fetchWidth) map { i =>
      f3_mask(i) && f3_preds(i).predicted_pc.valid &&
      (f3_preds(i).is_jal || (f3_preds(i).is_br && f3_preds(i).taken))
    }
    val f3_redirect_idx = PriorityEncoder(f3_redirects)
    val f3_targs = f3_preds zip f3_is_ret map { case (p, is_ret) =>
                    Mux(is_ret && useRAS.B, p.ras_top, p.predicted_pc.bits) }
    val f3_do_redirect = f3_redirects.reduce(_||_) && useBPD.B
    val f3_predicted_target = Mux(f3_do_redirect,
                                  f3_targs(f3_redirect_idx),
                                  nextFetch(s3_vpc))
    val (f3_predicted_ghist, f3_pred_ghist_update_type) = s3_ghist.update(
      f3_preds.map(p => p.is_br && p.predicted_pc.valid).asUInt & f3_mask,
      f3_preds(f3_redirect_idx).taken && f3_do_redirect,
      f3_preds(f3_redirect_idx).is_br,
      f3_redirect_idx,
      f3_do_redirect,
      s3_vpc,
      f3_is_call(f3_redirect_idx),
      f3_is_ret(f3_redirect_idx))   
    
    // 输出 f3 的 meta info
    io.resp.f3_meta.ghist := s3_ghist
    io.resp.f3_meta.pc := s3_vpc    
    io.resp.f3_meta.meta(0) := banked_predictors(0).io.f3_meta
    io.resp.f3_meta.tsrc := s3_tsrc
    io.resp.f3_meta.fsrc := s3_fsrc

    // 输出 f3 的 preds info
    val f3_br_taken = f3_preds.map(p =>
      Mux(disablePostCondCorrection.B,
        p.taken && p.predicted_pc.valid && p.is_br,
        p.taken))
    io.resp.f3_preds_info.br_taken := VecInit(f3_br_taken).asUInt
    val f3_is_jal = (0 until fetchWidth) map { i =>
      f3_mask(i) && f3_preds(i).predicted_pc.valid && (f3_preds(i).is_jal)
    }
    val f3_jal_idx = PriorityEncoder(f3_is_jal)
    val f3_jal_target = f3_preds(f3_jal_idx).predicted_pc.bits
    io.resp.f3_preds_info.jal_target := f3_jal_target
    io.resp.f3_preds_info.jal_targets_debug := f3_preds.map(p => p.predicted_pc.bits)
    io.resp.f3_preds_info.ras_top := f3_preds(0).ras_top
    io.resp.f3_preds_info.ras_idx := f2_ras_top_idx
    io.resp.f3_preds_info.btb_hits := f3_preds.map(p => p.predicted_pc.valid).asUInt
    io.resp.f3_preds_target.f2_pred_target := f3_last_pred_target
    io.resp.f3_preds_target.f2_pred_ghist_update_type := f3_ghist_update_type
    io.resp.f3_preds_target.f3_pred_target := f3_predicted_target
    io.resp.f3_preds_target.f3_pred_ghist_update_type := f3_pred_ghist_update_type


    // 输出 f3 的预测结果
    io.resp.f3_pred_valid := s3_valid
    io.resp.f3_next_ghist := f3_predicted_ghist
    io.resp.f3_next_pc := f3_predicted_target
    
    // f3 重定向 f1/f2
    val f3_ghist_all_zero = s3_ghist === (0.U).asTypeOf(new GlobalHistory)
    val shift_zero_or_no_shift_f3 = f3_pred_ghist_update_type(1) === 0.U && f3_ghist_update_type(1) === 0.U
    val f3_correct_ghist = !(f3_ghist_all_zero && shift_zero_or_no_shift_f3) &&
                            f3_pred_ghist_update_type =/= f3_ghist_update_type &&
                            enableGHistStallRepair.B

    val f3_do_redirection = f3_correct_ghist || f3_predicted_target =/= f3_last_pred_target

    io.resp.f3_ftq_idx := s3_ftq_idx
    io.resp.f3_redirect := false.B
    when (s3_valid && f3_do_redirection) {
      io.resp.f3_redirect := true.B
    }

    // f3 RAS 更新
    when (s3_valid && f3_do_redirect) {
      when (f3_is_ret(f3_redirect_idx)) {
        // ret 指令，pop RAS
        f3_ras_top_update_valid := true.B
        f3_ras_top_update_idx := WrapDec(f2_ras_top_idx, nRasEntries)
        
        io.f3_cfi_is_ret_debug := true.B
        io.f3_cfi_ret_addr_debug := bankAlign(s3_vpc) + (f3_redirect_idx << 1)
      } .elsewhen (f3_is_call(f3_redirect_idx)) {
        // call 指令，push RAS
        f3_ras_update_valid := true.B
        f3_ras_update_idx := WrapInc(f2_ras_top_idx, nRasEntries)
        f3_ras_update_addr := bankAlign(s3_vpc) + (f3_redirect_idx << 1) + Mux(
                              f3_preds(f3_redirect_idx).npc_plus2, 2.U, 4.U)
        // 更新 ras top 指针
        f3_ras_top_update_valid := true.B
        f3_ras_top_update_idx := WrapInc(f2_ras_top_idx, nRasEntries)


        io.f3_cfi_is_call_debug := true.B
        io.f3_cfi_call_addr_debug := bankAlign(s3_vpc) + (f3_redirect_idx << 1)
      }
    }

    // Assertion
    assert (!(s2_valid && RegNext(io.resp.f2_redirect, false.B)), 
    "s2_valid should be false if last cycle redirected")
    // TODO: f3_redirect 可能被 predecode 否决
    // assert (!(s2_valid && RegNext(io.resp.f2_redirect || io.resp.f3_redirect, false.B)), 
    // "s2_valid should be false if last cycle redirected")
    // assert (!(s3_valid && RegNext(io.resp.f3_redirect, false.B)),
    // "s3_valid should be false if last cycle redirected")
    // assert (!(s3_valid && RegNext(RegNext(io.resp.f3_redirect, false.B), false.B)),
    // "s3_valid should be false if redirected two cycles ago")
    when (s3_valid) {
      // 检测 jal/jalr 指令是否正确处理了跳转以及跳转目标
      val f3_is_jal = (0 until fetchWidth) map { i =>
        f3_preds(i).predicted_pc.valid && f3_mask(i) && (f3_preds(i).is_jal) && (!f3_preds(i).is_ret)
      }
      val f3_has_jal = f3_is_jal.reduce(_||_)
      val f3_jal_idx = PriorityEncoder(f3_is_jal)
      assert (f3_do_redirect || !f3_is_jal.reduce(_||_), 
        "If there is a jal in f3, f3_do_redirect should be true")
      assert ( !f3_has_jal || (f3_redirect_idx <= f3_jal_idx), 
        "If there is a jal in f3, f3_redirect_idx should point to the jal or an earlier instruction")
      assert ( !f3_has_jal || f3_redirect_idx =/= f3_jal_idx ||
        f3_predicted_target === f3_preds(f3_jal_idx).predicted_pc.bits,
        "If f3_redirect_idx points to a jal, the predicted target should equal the predecode target")
      
      // 检测 ret 指令是否正确处理了跳转以及跳转目标
      when (useBPD.B && useRAS.B) {
        val f3_ret_masks = (f3_is_ret zip f3_mask.asBools) map {case (is_ret, m) => is_ret && m}
        val f3_has_ret = f3_ret_masks.reduce(_||_)
        val f3_ret_idx = PriorityEncoder(f3_ret_masks)
        assert (f3_do_redirect || !f3_ret_masks.reduce(_||_), 
          "If there is a ret in f3, f3_do_redirect should be true")
        assert ( !f3_has_ret || (f3_redirect_idx <= f3_ret_idx), 
          "If there is a ret in f3, f3_redirect_idx should point to the ret or an earlier instruction")
        assert ( !f3_has_ret || f3_redirect_idx =/= f3_ret_idx ||
          f3_predicted_target === f3_preds(f3_ret_idx).ras_top,
          "If f3_redirect_idx points to a ret, the predicted target should equal the RAS top")
      }
      when (useBPD.B) {
        // 检测 br 指令是否正确处理了跳转以及跳转目标
        val f3_taken_br_masks = (f3_mask.asBools zip f3_preds) map 
                                {case (m, p) => p.predicted_pc.valid && p.is_br && p.taken && m}
        val f3_has_taken_br = f3_taken_br_masks.reduce(_||_)
        val f3_taken_br_idx = PriorityEncoder(f3_taken_br_masks)
        assert (f3_do_redirect || !f3_taken_br_masks.reduce(_||_), 
          "If there is a taken br in f3, f3_do_redirect should be true")
        assert ( !f3_has_taken_br || (f3_redirect_idx <= f3_taken_br_idx), 
          "If there is a taken br in f3, f3_redirect_idx should point to the taken br or an earlier instruction")
        assert ( !f3_has_taken_br || f3_redirect_idx =/= f3_taken_br_idx ||
          f3_predicted_target === f3_preds(f3_taken_br_idx).predicted_pc.bits,
          "If f3_redirect_idx points to a taken br, the predicted target should equal the predicted target")
      }
    }

  } else {
    require(nBanks == 2)
    val b0_fire = io.f3_fire && RegNext(RegNext(RegNext(banked_predictors(0).io.f0_valid)))
    val b1_fire = io.f3_fire && RegNext(RegNext(RegNext(banked_predictors(1).io.f0_valid)))
    banked_predictors(0).io.f3_fire := b0_fire
    banked_predictors(1).io.f3_fire := b1_fire

    banked_lhist_providers(0).io.f3_fire := b0_fire
    banked_lhist_providers(1).io.f3_fire := b1_fire

  }


  for (i <- 0 until nBanks) {
    banked_predictors(i).io.update.bits.is_mispredict_update := io.update.bits.is_mispredict_update
    banked_predictors(i).io.update.bits.is_repair_update     := io.update.bits.is_repair_update

    banked_predictors(i).io.update.bits.meta             := io.update.bits.meta(i)
    banked_predictors(i).io.update.bits.lhist            := io.update.bits.lhist(i)
    banked_predictors(i).io.update.bits.cfi_idx.bits     := io.update.bits.cfi_idx.bits
    banked_predictors(i).io.update.bits.cfi_taken        := io.update.bits.cfi_taken
    banked_predictors(i).io.update.bits.cfi_mispredicted := io.update.bits.cfi_mispredicted
    banked_predictors(i).io.update.bits.cfi_is_br        := io.update.bits.cfi_is_br
    banked_predictors(i).io.update.bits.cfi_is_jal       := io.update.bits.cfi_is_jal
    banked_predictors(i).io.update.bits.cfi_is_jalr      := io.update.bits.cfi_is_jalr
    banked_predictors(i).io.update.bits.cfi_is_call      := io.update.bits.cfi_is_call
    banked_predictors(i).io.update.bits.cfi_is_ret       := io.update.bits.cfi_is_ret
    banked_predictors(i).io.update.bits.cfi_npc_plus4    := io.update.bits.cfi_npc_plus4
    banked_predictors(i).io.update.bits.target           := io.update.bits.target

    banked_lhist_providers(i).io.update.mispredict := io.update.bits.is_mispredict_update
    banked_lhist_providers(i).io.update.repair     := io.update.bits.is_repair_update
    banked_lhist_providers(i).io.update.lhist      := io.update.bits.lhist(i)
  }

  if (nBanks == 1) {
    banked_predictors(0).io.update.valid                 := io.update.valid
    banked_predictors(0).io.update.bits.pc               := bankAlign(io.update.bits.pc)
    banked_predictors(0).io.update.bits.br_mask          := io.update.bits.br_mask
    banked_predictors(0).io.update.bits.btb_mispredicts  := io.update.bits.btb_mispredicts
    banked_predictors(0).io.update.bits.cfi_idx.valid    := io.update.bits.cfi_idx.valid
    banked_predictors(0).io.update.bits.ghist            := io.update.bits.ghist.histories(0)

    banked_lhist_providers(0).io.update.valid := io.update.valid && io.update.bits.br_mask =/= 0.U
    banked_lhist_providers(0).io.update.pc    := bankAlign(io.update.bits.pc)
  } else {
    require(nBanks == 2)
    // Split the single update bundle for the fetchpacket into two updates
    // 1 for each bank.

    when (bank(io.update.bits.pc) === 0.U) {
      val b1_update_valid = io.update.valid &&
        (!io.update.bits.cfi_idx.valid || io.update.bits.cfi_idx.bits >= bankWidth.U)

      banked_lhist_providers(0).io.update.valid := io.update.valid && io.update.bits.br_mask(bankWidth-1,0) =/= 0.U
      banked_lhist_providers(1).io.update.valid := b1_update_valid && io.update.bits.br_mask(fetchWidth-1,bankWidth) =/= 0.U

      banked_lhist_providers(0).io.update.pc := bankAlign(io.update.bits.pc)
      banked_lhist_providers(1).io.update.pc := nextBank(io.update.bits.pc)

      banked_predictors(0).io.update.valid := io.update.valid
      banked_predictors(1).io.update.valid := b1_update_valid

      banked_predictors(0).io.update.bits.pc := bankAlign(io.update.bits.pc)
      banked_predictors(1).io.update.bits.pc := nextBank(io.update.bits.pc)

      banked_predictors(0).io.update.bits.br_mask := io.update.bits.br_mask
      banked_predictors(1).io.update.bits.br_mask := io.update.bits.br_mask >> bankWidth

      banked_predictors(0).io.update.bits.btb_mispredicts  := io.update.bits.btb_mispredicts
      banked_predictors(1).io.update.bits.btb_mispredicts  := io.update.bits.btb_mispredicts >> bankWidth

      banked_predictors(0).io.update.bits.cfi_idx.valid := io.update.bits.cfi_idx.valid && io.update.bits.cfi_idx.bits < bankWidth.U
      banked_predictors(1).io.update.bits.cfi_idx.valid := io.update.bits.cfi_idx.valid && io.update.bits.cfi_idx.bits >= bankWidth.U

      banked_predictors(0).io.update.bits.ghist := io.update.bits.ghist.histories(0)
      banked_predictors(1).io.update.bits.ghist := io.update.bits.ghist.histories(1)
    } .otherwise {
      val b0_update_valid = io.update.valid && !mayNotBeDualBanked(io.update.bits.pc) &&
        (!io.update.bits.cfi_idx.valid || io.update.bits.cfi_idx.bits >= bankWidth.U)

      banked_lhist_providers(1).io.update.valid := io.update.valid && io.update.bits.br_mask(bankWidth-1,0) =/= 0.U
      banked_lhist_providers(0).io.update.valid := b0_update_valid && io.update.bits.br_mask(fetchWidth-1,bankWidth) =/= 0.U

      banked_lhist_providers(1).io.update.pc := bankAlign(io.update.bits.pc)
      banked_lhist_providers(0).io.update.pc := nextBank(io.update.bits.pc)

      banked_predictors(1).io.update.valid := io.update.valid
      banked_predictors(0).io.update.valid := b0_update_valid

      banked_predictors(1).io.update.bits.pc := bankAlign(io.update.bits.pc)
      banked_predictors(0).io.update.bits.pc := nextBank(io.update.bits.pc)

      banked_predictors(1).io.update.bits.br_mask := io.update.bits.br_mask
      banked_predictors(0).io.update.bits.br_mask := io.update.bits.br_mask >> bankWidth

      banked_predictors(1).io.update.bits.btb_mispredicts  := io.update.bits.btb_mispredicts
      banked_predictors(0).io.update.bits.btb_mispredicts  := io.update.bits.btb_mispredicts >> bankWidth

      banked_predictors(1).io.update.bits.cfi_idx.valid := io.update.bits.cfi_idx.valid && io.update.bits.cfi_idx.bits < bankWidth.U
      banked_predictors(0).io.update.bits.cfi_idx.valid := io.update.bits.cfi_idx.valid && io.update.bits.cfi_idx.bits >= bankWidth.U

      banked_predictors(1).io.update.bits.ghist := io.update.bits.ghist.histories(0)
      banked_predictors(0).io.update.bits.ghist := io.update.bits.ghist.histories(1)
    }

  }

  when (io.update.valid) {
    when (io.update.bits.cfi_is_br && io.update.bits.cfi_idx.valid) {
      assert(io.update.bits.br_mask(io.update.bits.cfi_idx.bits))
    }
  }
}

class NullBranchPredictorBank(implicit p: Parameters) extends BranchPredictorBank()(p) {
  val mems = Nil
}

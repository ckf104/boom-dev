package boom.v3.ifu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.{Field, Parameters}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util.PlusArg
import freechips.rocketchip.util._

import boom.v3.common._
import boom.v3.util.{BoomCoreStringPrefix}

import scala.math.min

case class BoomBTBParams(
  nSets: Int = 128,
  nWays: Int = 2,
  offsetSz: Int = 13,
  extendedNSets: Int = 128
)

case object BoomBTBKey extends Field[BoomBTBParams](BoomBTBParams())

class BTBBranchPredictorBank(params: BoomBTBParams = BoomBTBParams())(implicit p: Parameters) extends BranchPredictorBank()(p)
{
  override val nSets         = params.nSets
  override val nWays         = params.nWays
  val tagSz         = vaddrBitsExtended - log2Ceil(nSets) - log2Ceil(fetchWidth) - 1
  val offsetSz      = params.offsetSz
  val extendedNSets = params.extendedNSets

  require(isPow2(nSets))
  require(isPow2(extendedNSets) || extendedNSets == 0)
  require(extendedNSets <= nSets)
  require(extendedNSets >= 1)

  class BTBEntry extends Bundle {
    val offset   = SInt(offsetSz.W)
    val extended = Bool()
  }
  val btbEntrySz = offsetSz + 1

  class BTBMeta extends Bundle {
    val is_br = Bool()
    val is_call = Bool()
    val is_ret  = Bool()
    val tag   = UInt(tagSz.W)
  }
  val btbMetaSz = tagSz + 3

  class BTBPredictMeta extends Bundle {
    val write_way = UInt(log2Ceil(nWays).W)
    // 用于 btb fix
    val hit_ways = Vec(bankWidth, UInt(log2Ceil(nWays).W))
  }

  val s1_meta = Wire(new BTBPredictMeta)
  val f3_meta = RegNext(RegNext(s1_meta))


  io.f3_meta := f3_meta.asUInt

  override val metaSz = s1_meta.asUInt.getWidth

  val doing_reset = RegInit(true.B)
  val reset_idx   = RegInit(0.U(log2Ceil(nSets).W))
  reset_idx := reset_idx + doing_reset
  when (reset_idx === (nSets-1).U) { doing_reset := false.B }

  val meta     = Seq.fill(nWays) { SyncReadMem(nSets, Vec(bankWidth, UInt(btbMetaSz.W))) }
  val btb      = Seq.fill(nWays) { SyncReadMem(nSets, Vec(bankWidth, UInt(btbEntrySz.W))) }
  val ebtb     = SyncReadMem(extendedNSets, UInt(vaddrBitsExtended.W))

  val mems = (((0 until nWays) map ({w:Int => Seq(
    (f"btb_meta_way$w", nSets, bankWidth * btbMetaSz),
    (f"btb_data_way$w", nSets, bankWidth * btbEntrySz))})).flatten ++ Seq(("ebtb", extendedNSets, vaddrBitsExtended)))

  val s1_req_rbtb  = VecInit(btb.map { b => VecInit(b.read(s0_idx , s0_valid).map(_.asTypeOf(new BTBEntry))) })
  val s1_req_rmeta = VecInit(meta.map { m => VecInit(m.read(s0_idx, s0_valid).map(_.asTypeOf(new BTBMeta))) })
  val s1_req_rebtb = ebtb.read(s0_idx, s0_valid)
  val s1_req_tag   = s1_idx >> log2Ceil(nSets)

  val s1_resp   = Wire(Vec(bankWidth, Valid(UInt(vaddrBitsExtended.W))))
  val s1_is_br  = Wire(Vec(bankWidth, Bool()))
  val s1_is_jal = Wire(Vec(bankWidth, Bool()))
  val s1_is_call = Wire(Vec(bankWidth, Bool()))
  val s1_is_ret  = Wire(Vec(bankWidth, Bool()))

  val s1_hit_ohs = VecInit((0 until bankWidth) map { i =>
    VecInit((0 until nWays) map { w =>
      s1_req_rmeta(w)(i).tag === s1_req_tag(tagSz-1,0)
    })
  })
  val s1_hits     = s1_hit_ohs.map { oh => oh.reduce(_||_) }
  val s1_hit_ways = s1_hit_ohs.map { oh => PriorityEncoder(oh) }

  for (w <- 0 until bankWidth) {
    val entry_meta = s1_req_rmeta(s1_hit_ways(w))(w)
    val entry_btb  = s1_req_rbtb(s1_hit_ways(w))(w)
    s1_resp(w).valid := !doing_reset && s1_valid && s1_hits(w)
    s1_resp(w).bits  := Mux(
      entry_btb.extended,
      s1_req_rebtb,
      (s1_pc.asSInt + (w << 1).S + entry_btb.offset).asUInt)
    s1_is_br(w)  := !doing_reset && s1_resp(w).valid &&  entry_meta.is_br
    s1_is_jal(w) := !doing_reset && s1_resp(w).valid && !entry_meta.is_br
    s1_is_call(w) := !doing_reset && s1_resp(w).valid && entry_meta.is_call
    s1_is_ret(w)  := !doing_reset && s1_resp(w).valid && entry_meta.is_ret


    io.resp.f2(w) := io.resp_in(0).f2(w)
    io.resp.f3(w) := io.resp_in(0).f3(w)
    when (RegNext(s1_hits(w))) {
      io.resp.f2(w).predicted_pc := RegNext(s1_resp(w))
      io.resp.f2(w).is_br        := RegNext(s1_is_br(w))
      io.resp.f2(w).is_jal       := RegNext(s1_is_jal(w))
      io.resp.f2(w).is_call      := RegNext(s1_is_call(w))
      io.resp.f2(w).is_ret       := RegNext(s1_is_ret(w))
      // s1_is_jal 是 !s1_is_br，所以它表示该指令是否是无条件跳转指令
      // 包括了 jal 指令和 jalr 指令
      when (RegNext(s1_is_jal(w))) {
        io.resp.f2(w).taken      := true.B
      }
    }
    when (RegNext(RegNext(s1_hits(w)))) {
      io.resp.f3(w).predicted_pc := RegNext(io.resp.f2(w).predicted_pc)
      io.resp.f3(w).is_br        := RegNext(io.resp.f2(w).is_br)
      io.resp.f3(w).is_jal       := RegNext(io.resp.f2(w).is_jal)
      io.resp.f3(w).is_call      := RegNext(io.resp.f2(w).is_call)
      io.resp.f3(w).is_ret       := RegNext(io.resp.f2(w).is_ret)
      when (RegNext(RegNext(s1_is_jal(w)))) {
        io.resp.f3(w).taken      := true.B
      }
    }
  }

  val alloc_way = if (nWays > 1) {
    val r_metas = Cat(VecInit(s1_req_rmeta.map { w => VecInit(w.map(_.tag)) }).asUInt, s1_req_tag(tagSz-1,0))
    val l = log2Ceil(nWays)
    val nChunks = (r_metas.getWidth + l - 1) / l
    val chunks = (0 until nChunks) map { i =>
      r_metas(min((i+1)*l, r_metas.getWidth)-1, i*l)
    }
    chunks.reduce(_^_)
  } else {
    0.U
  }
  s1_meta.write_way := Mux(s1_hits.reduce(_||_),
    PriorityEncoder(s1_hit_ohs.map(_.asUInt).reduce(_|_)),
    alloc_way)
  s1_meta.hit_ways := s1_hit_ways

  val s1_update_cfi_idx = s1_update.bits.cfi_idx.bits
  val s1_update_meta    = s1_update.bits.meta.asTypeOf(new BTBPredictMeta)

  val max_offset_value = Cat(0.B, ~(0.U((offsetSz-1).W))).asSInt
  val min_offset_value = Cat(1.B,  (0.U((offsetSz-1).W))).asSInt
  val new_offset_value = (s1_update.bits.target.asSInt -
    (s1_update.bits.pc + (s1_update.bits.cfi_idx.bits << 1)).asSInt)
  val offset_is_extended = (new_offset_value > max_offset_value ||
                            new_offset_value < min_offset_value)


  val s1_update_wbtb_data  = Wire(new BTBEntry)
  s1_update_wbtb_data.extended := offset_is_extended
  s1_update_wbtb_data.offset   := new_offset_value
  val s1_update_wbtb_mask = (UIntToOH(s1_update_cfi_idx) &
    Fill(bankWidth, s1_update.bits.cfi_idx.valid && s1_update.valid && s1_update.bits.cfi_taken && s1_update.bits.is_commit_update))

  val commit_update_mask = (s1_update_wbtb_mask | s1_update.bits.br_mask) &
                            Fill(bankWidth, s1_update.valid && s1_update.bits.is_commit_update)
  val mispredict_update_mask = Wire(Vec(nWays, UInt(bankWidth.W)))
  for (w <- 0 until nWays) {
    mispredict_update_mask(w) := Cat((0 until bankWidth).reverse.map { b =>
      s1_meta.hit_ways(b) === w.U &&
      s1_update.bits.btb_mispredicts(b) && enableBTBFastRepair.B &&
      s1_update.valid && s1_update.bits.is_commit_update
    })
  }
  val s1_update_wmeta_mask = Wire(Vec(nWays, UInt(bankWidth.W)))
  for (w <- 0 until nWays) {
    when (s1_update_meta.write_way === w.U) {
      s1_update_wmeta_mask(w) := commit_update_mask | mispredict_update_mask(w)
    } .otherwise {
      s1_update_wmeta_mask(w) := mispredict_update_mask(w)
    }
  }

  val s1_update_wmeta_data = WireDefault(VecInit.fill(nWays)(VecInit.fill(bankWidth)(0.U.asTypeOf(new BTBMeta))))

  for (w <- 0 until bankWidth) {
    s1_update_wmeta_data(s1_update_meta.write_way)(w).tag     := Mux(commit_update_mask(w), s1_update_idx >> log2Ceil(nSets), 0.U)
    s1_update_wmeta_data(s1_update_meta.write_way)(w).is_br   := s1_update.bits.br_mask(w)
    s1_update_wmeta_data(s1_update_meta.write_way)(w).is_call := s1_update.bits.cfi_idx.valid && s1_update_cfi_idx === w.U && s1_update.bits.cfi_is_call
    s1_update_wmeta_data(s1_update_meta.write_way)(w).is_ret  := s1_update.bits.cfi_idx.valid && s1_update_cfi_idx === w.U && s1_update.bits.cfi_is_ret
  }

  val btb_reset_value = WireDefault(0.U(btbEntrySz.W))
  val meta_reset_value = WireDefault(0.U(btbMetaSz.W))
  if (IN_SIMULATION) {
    val cmd_btb_reset_v = PlusArg("btb-reset-value", 0x0, "Value to initialize BTB entries to on reset", offsetSz)
    btb_reset_value := Cat(cmd_btb_reset_v, 0.U(1.W))
    val cmd_meta_reset_v = PlusArg("btb-meta-reset-value", 0x0, "Value to initialize BTB meta entries to on reset", btbMetaSz)
    meta_reset_value := cmd_meta_reset_v
  }

  if (IN_SIMULATION) {
    // Add a free-running cycle counter for simulation
    val (cycleCount, _) = Counter(true.B, Int.MaxValue)
    val btb_access_printf = PlusArg("f2btb-access-printf", 0, "Enable F2 BTB access printf debugging", 1)
    val s2_vpc  = RegNext(s1_pc)
    val s2_idx  = RegNext(s1_idx)
    val s2_tag  = RegNext(s1_req_tag)    
    val s2_hit = RegNext(s1_hits.reduce(_||_))
    when (s2_hit && btb_access_printf(0) && !doing_reset) {
      // F2 阶段 BTB 命中，打印本次取指的 BTB 命中信息
      printf("[%d F2 BTB] S1 PC: 0x%x, idx: 0x%x, tag: 0x%x\n",
        cycleCount,
        s2_vpc,
        s2_idx(log2Ceil(nSets)-1,0),
        s2_tag(tagSz-1,0)
      )
      // 打印每个 slot 的 F2 预测结果
      for (w <- 0 until bankWidth) {
        when (RegNext(s1_hits(w))) {
          printf("  slot=%d valid=%d pred_pc=0x%x is_br=%d is_jal=%d is_call=%d is_ret=%d taken=%d\n",
            w.U,
            io.resp.f2(w).predicted_pc.valid,
            io.resp.f2(w).predicted_pc.bits,
            io.resp.f2(w).is_br,
            io.resp.f2(w).is_jal,
            io.resp.f2(w).is_call,
            io.resp.f2(w).is_ret,
            io.resp.f2(w).taken
          )
        }
      }
    }

    val btb_update_printf = PlusArg("f2btb-update-printf", 0, "Enable F2 BTB update printf debugging", 1)
    when (s1_update.valid && s1_update.bits.is_commit_update && !doing_reset && btb_update_printf(0)) {
      printf("[%d F2 BTB Update] PC: 0x%x, idx: 0x%x, tag: 0x%x, cfi_valid: %b, cfi_idx: %d, target: 0x%x," +
      " taken: %b, br_mask: %b, cfi_is_call: %b, cfi_is_ret: %b, cfi_is_jal: %b, btb_mispredicts: %b," +
      " write_way: %d, new_offset: 0x%x (extended: %b)\n",
        cycleCount,
        s1_update.bits.pc,
        s1_update_idx(log2Ceil(nSets)-1,0),
        s1_update_idx >> log2Ceil(nSets),
        s1_update.bits.cfi_idx.valid,
        s1_update_cfi_idx,
        s1_update.bits.target,
        s1_update.bits.cfi_taken,
        s1_update.bits.br_mask.asUInt,
        s1_update.bits.cfi_is_call,
        s1_update.bits.cfi_is_ret,
        s1_update.bits.cfi_is_jal,
        s1_update.bits.btb_mispredicts.asUInt,
        s1_update_meta.write_way,
        new_offset_value,
        offset_is_extended
      )
    }
    val btb_mis_update_printf = PlusArg("f2btb-mis-update-printf", 0, "Enable F2 BTB mispredict update printf debugging", 1)
    when (s1_update.valid && s1_update.bits.is_commit_update && s1_update.bits.btb_mispredicts.asUInt =/= 0.U &&
          !doing_reset && btb_mis_update_printf(0)) {
      printf("[%d F2 BTB Mispredict Update] PC: 0x%x, idx: 0x%x, tag: 0x%x, btb_mispredicts: %b, hit_ways: %b, write_way: %d\n",
        cycleCount,
        s1_update.bits.pc,
        s1_update_idx(log2Ceil(nSets)-1,0),
        s1_update_idx >> log2Ceil(nSets),
        s1_update.bits.btb_mispredicts.asUInt,
        s1_update_meta.hit_ways.asUInt,
        s1_update_meta.write_way
      )
    }
  }

  for (w <- 0 until nWays) {
    when (doing_reset || s1_update_meta.write_way === w.U) {
      btb(w).write(
        Mux(doing_reset,
          reset_idx,
          s1_update_idx),
        Mux(doing_reset,
          VecInit(Seq.fill(bankWidth) { btb_reset_value }),
          VecInit(Seq.fill(bankWidth) { s1_update_wbtb_data.asUInt })),
        Mux(doing_reset,
          (~(0.U(bankWidth.W))),
          s1_update_wbtb_mask).asBools
      )
    }
  }
  for (w <- 0 until nWays) {
      meta(w).write(
        Mux(doing_reset,
          reset_idx,
          s1_update_idx),
        Mux(doing_reset,
          VecInit(Seq.fill(bankWidth) { meta_reset_value }),
          VecInit(s1_update_wmeta_data(w).map(_.asUInt))),
        Mux(doing_reset,
          (~(0.U(bankWidth.W))),
          s1_update_wmeta_mask(w)).asBools
      )
  }
  when (s1_update_wbtb_mask =/= 0.U && offset_is_extended) {
    ebtb.write(s1_update_idx, s1_update.bits.target)
  }

  // Assertions
  for (w <- 0 until bankWidth) {
    when (io.resp.f2(w).predicted_pc.valid) {
      assert(!(io.resp.f2(w).is_jal && io.resp.f2(w).is_br),
        p"BTB predicted PC cannot be both JAL and BR\n")
      assert(!(io.resp.f2(w).is_call && !io.resp.f2(w).is_jal),
        p"BTB predicted PC cannot be CALL without JAL\n")
      assert(!(io.resp.f2(w).is_ret && !io.resp.f2(w).is_jal),
        p"BTB predicted PC cannot be RET without JAL\n")
      assert(!(io.resp.f2(w).is_call && io.resp.f2(w).is_ret),
        p"BTB predicted PC cannot be both CALL and RET\n")
    }
    when (io.resp.f3(w).predicted_pc.valid) {
      assert(!(io.resp.f3(w).is_jal && io.resp.f3(w).is_br),
        p"BTB predicted PC cannot be both JAL and BR\n")
      assert(!(io.resp.f3(w).is_call && !io.resp.f3(w).is_jal),
        p"BTB predicted PC cannot be CALL without JAL\n")
      assert(!(io.resp.f3(w).is_ret && !io.resp.f3(w).is_jal),
        p"BTB predicted PC cannot be RET without JAL\n")
      assert(!(io.resp.f3(w).is_call && io.resp.f3(w).is_ret),
        p"BTB predicted PC cannot be both CALL and RET\n")
    }
    when (s1_update.valid && s1_update.bits.is_commit_update && !doing_reset) {
      assert(!(s1_update.bits.cfi_idx.valid && s1_update.bits.cfi_is_br && s1_update.bits.cfi_is_jal),
        p"cfi cannot be both JAL and BR\n")
      assert(!(s1_update.bits.cfi_idx.valid && s1_update.bits.cfi_is_jal && !s1_update.bits.cfi_taken),
        p"cfi should be taken if it is a JAL\n")
      assert(!(s1_update.bits.cfi_idx.valid && s1_update.bits.cfi_is_call && !s1_update.bits.cfi_is_jal),
        p"cfi cannot be CALL without being JAL\n")
      assert(!(s1_update.bits.cfi_idx.valid && s1_update.bits.cfi_is_ret && !s1_update.bits.cfi_is_jal),
        p"cfi cannot be RET without being JAL\n")
      assert(!(s1_update.bits.cfi_idx.valid && s1_update.bits.cfi_is_call && s1_update.bits.cfi_is_ret),
        p"cfi cannot be both CALL and RET\n")
    }
  }
  when (s1_update.valid) {
    assert ((commit_update_mask & s1_update.bits.btb_mispredicts) === 0.U,
      p"BTB update can't commit update and mispredict update the same bank\n")
  }
}

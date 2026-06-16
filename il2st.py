#!/usr/bin/env python3
"""
il2st.py
========
Mitsubishi FX5U Instruction List (IL) -> OpenPLC v3 Structured Text (ST) Converter.

Scope:
    Instruction set covered in Lab 01 through Lab 15:
      Boolean:   LD, LDI, AND, ANI, OR, ORI, OUT, END
      Block:     ANB, ORB
      Stack:     MPS, MRD, MPP
      Edge:      PLS, PLF
      Latch:     SET, RST
      Timer:     OUT Tn Kxx | OUT Tn Dxx
      Counter:   OUT Cn Kxx, RST Cn
      Step:      STL, RET (linear and parallel divergence/convergence auto-detected)
      Data:      MOV, INCP, ZRST, CMP
      Special M: SM8000/M8000, SM8002/M8002, SM8013/M8013

Translation philosophy: LITERAL (one IL line -> one ST line/block where possible).
No pattern compression (e.g. XOR collapse). That comes later.

Output guarantees (project constitution):
  * Single ASCII space between every token in located VAR decls
  * Located vars never share a VAR block with FB instances
  * Reserved-word collisions are auto-renamed
  * Full PROGRAM ... END_PROGRAM + CONFIGURATION Config0 / RESOURCE Res0 skeleton
  * Address mapping by original index (X3 -> %QX100.3, not the 2nd X seen)
  * Lint pass validates the monitoring.py single-space rule before write

Usage:
    CLI:    python il2st.py <input.csv> [-o output.st] [-n ProgName] [--task-ms 20]
    Module: from il2st import convert
            st_text = convert(csv_text, program_name='Prac01')

CSV input format (fixed):
    Three columns -- step, instr, operand(s).
    Header row optional.
    The third column may pack multiple tokens separated by whitespace
    (e.g. "T0 K10" for `OUT T0 K10`).
    A fourth column, if present, is treated as an additional operand.

Author: built for sh0ut's IL->ST conversion project.
"""

from __future__ import annotations

import argparse
import csv
import re
import sys
from collections import OrderedDict, defaultdict
from dataclasses import dataclass, field
from io import StringIO
from typing import Optional


# ---------------------------------------------------------------------------
# 1. Constants / configuration
# ---------------------------------------------------------------------------

K_PER_TICK_MS = 100  # FX5U default timer base = 100 ms

# ---------------------------------------------------------------------------
# Address layout (OpenPLC %QX byte offsets)
# ---------------------------------------------------------------------------
# X/Y are fixed by the HMI-loopback convention (X->%QX0-1, Y->%QX2-3). The
# internal zones (M, S, SM) are spaced far apart so that even large index
# ranges never overlap each other. Each zone gets its own contiguous block:
#
#   X    bytes 0..1     X0..X17 (octal)        Modbus coil 0..15
#   Y    bytes 2..3     Y0..Y17 (octal)        Modbus coil 16..31
#   M    bytes 110..199 M0..M719               (90 bytes)
#   S    bytes 200..399 S0..S1599              (200 bytes; covers S0..S999 easily)
#   SM   bytes 400..    SM8002/SM8013
#
# A lint pass also checks for any residual address overlap as a safety net.
ADDR_M_BASE = 110
ADDR_S_BASE = 200
ADDR_SM_BASE = 400
ADDR_M_MAX_BYTE = ADDR_S_BASE - 1    # M must stay below S
ADDR_S_MAX_BYTE = ADDR_SM_BASE - 1   # S must stay below SM

ALLOWED_INSTRS = {
    # Boolean
    'LD', 'LDI', 'AND', 'ANI', 'OR', 'ORI', 'OUT', 'END',
    # Block / stack
    'ANB', 'ORB',
    'MPS', 'MRD', 'MPP',
    # Edge variants (P=rising, F=falling)
    'PLS', 'PLF',
    'LDP', 'LDF', 'ANDP', 'ANDF', 'ORP', 'ORF',
    # Latch
    'SET', 'RST',
    # Step
    'STL', 'RET',
    # Data
    'MOV', 'INC', 'INCP', 'DEC', 'DECP', 'ZRST', 'CMP',
    # Comparison contacts (LD<= D0 K0 style)
    'LD=', 'LD<>', 'LD<', 'LD>', 'LD<=', 'LD>=',
    'AND=', 'AND<>', 'AND<', 'AND>', 'AND<=', 'AND>=',
    'OR=', 'OR<>', 'OR<', 'OR>', 'OR<=', 'OR>=',
}

# Helper: maps `LD<=` -> ('LD', '<='), so we can dispatch.
_CMP_OPS = {'=', '<>', '<', '>', '<=', '>='}


def _split_cmp_op(op: str) -> Optional[tuple[str, str]]:
    """If op is a comparison-contact instruction like 'LD<=' or 'AND<', return
    (base, cmp_operator). Otherwise return None."""
    for cmp in sorted(_CMP_OPS, key=len, reverse=True):
        if op.endswith(cmp) and op[: -len(cmp)] in ('LD', 'AND', 'OR'):
            return (op[: -len(cmp)], cmp)
    return None

# Reserved words (case-insensitive). Variables colliding get auto-renamed.
RESERVED_WORDS = {
    # IEC 61131-3 ST core
    'program', 'end_program', 'function', 'end_function',
    'function_block', 'end_function_block',
    'configuration', 'end_configuration', 'resource', 'end_resource',
    'var', 'var_input', 'var_output', 'var_in_out', 'var_global',
    'var_external', 'var_temp', 'end_var',
    'constant', 'retain', 'non_retain',
    'at', 'with', 'task', 'interval', 'priority',
    'if', 'then', 'elsif', 'else', 'end_if',
    'case', 'of', 'end_case',
    'for', 'to', 'by', 'do', 'end_for',
    'while', 'end_while',
    'repeat', 'until', 'end_repeat',
    'exit', 'return',
    'true', 'false',
    'and', 'or', 'xor', 'not', 'mod',
    # SFC
    'step', 'end_step', 'initial_step',
    'transition', 'end_transition', 'action', 'end_action',
    'from', 'r_edge', 'f_edge', 'nil',
    # Types
    'type', 'end_type', 'struct', 'end_struct',
    'bool', 'byte', 'word', 'dword', 'lword',
    'sint', 'int', 'dint', 'lint',
    'usint', 'uint', 'udint', 'ulint',
    'real', 'lreal', 'time', 'date', 'tod', 'dt',
    'string', 'wstring',
    'time_of_day', 'date_and_time',
    # Standard FBs
    'ton', 'tof', 'tp',
    'ctu', 'ctd', 'ctud',
    'r_trig', 'f_trig',
    'sr', 'rs',
    # Standard funcs we use
    'int_to_time',
}

# Auto-rename map for variables that would clash with reserved words.
RENAME_MAP = {
    'step': 'state',
    't': 't_var',
    'd': 'd_var',
    's': 's_var',
    'int': 'int_var',
    'time': 'time_var',
}


# ---------------------------------------------------------------------------
# 2. Exceptions
# ---------------------------------------------------------------------------

class IL2STError(Exception):
    """Base error class."""
    pass


class CSVParseError(IL2STError):
    pass


class UnknownInstructionError(IL2STError):
    pass


class StackError(IL2STError):
    pass


class LintError(IL2STError):
    pass


# ---------------------------------------------------------------------------
# 3. Parsing -- CSV -> list[Instr]
# ---------------------------------------------------------------------------

@dataclass
class Instr:
    step: int           # original step number from CSV
    op: str             # uppercased mnemonic
    args: list          # list of raw operand strings (original case preserved)
    line_no: int        # CSV row number (1-indexed) for error messages

    def __repr__(self):
        return f"<{self.step}: {self.op} {' '.join(self.args)}>"


_STEP_RE = re.compile(r'^\d+$')


def _is_gxworks3_export(text: str) -> bool:
    """Heuristic: GX Works3 IL export starts with a Chinese title row and has
    a header containing 'Step No.' and is tab-delimited."""
    head = text[:2000]
    return ('Step No.' in head) and ('\t' in head)


def _normalize_gxworks3(text: str) -> str:
    """Convert GX Works3 7-col TSV export into the 3-col CSV format il2st
    expects. Joins multi-row operands (step blank + operand-only row) and
    drops the leading metadata rows."""
    rdr = csv.reader(StringIO(text), delimiter='\t')
    rows = [[c.strip() for c in row] for row in rdr]
    # find the header row
    header_idx = None
    for i, r in enumerate(rows):
        if r and r[0] == 'Step No.':
            header_idx = i
            break
    if header_idx is None:
        return text  # not actually GX-export format; fall back

    data_rows = rows[header_idx + 1:]

    # Convert into list of (step, instr, [operands])
    out_rows: list[tuple[int, str, list[str]]] = []
    for r in data_rows:
        if not any(c for c in r):
            continue
        # pad row to at least 4 cols
        while len(r) < 4:
            r.append('')
        step_cell = r[0]
        instr_cell = r[1] if len(r) > 1 else ''
        # GX export column layout: Step No. | Line Statement | Instruction | I/O (Device) | Blank | P/I | Note
        # so the 'instruction' is r[2], operand is r[3]
        instr_cell = r[2]
        operand_cell = r[3]
        if step_cell and _STEP_RE.match(step_cell):
            # new instruction row
            out_rows.append((int(step_cell), instr_cell,
                             [operand_cell] if operand_cell else []))
        else:
            # continuation row: operand-only, append to last instruction
            if out_rows and operand_cell:
                out_rows[-1][2].append(operand_cell)

    # Emit as 3-col CSV: step,instr,"op1 op2 op3"
    out = StringIO()
    w = csv.writer(out)
    w.writerow(['step', 'instr', 'operand'])
    for step, instr, opds in out_rows:
        operand = ' '.join(opds) if opds else ''
        w.writerow([step, instr, operand])
    return out.getvalue()


def parse_csv(text: str) -> list[Instr]:
    """Parse CSV text into a list of Instr.

    Accepts two formats:
      * 3-column CSV (step, instr, operand) -- our primary format.
      * GX Works3 IL export (UTF-16 TSV with 7 columns, multi-row operands) --
        auto-detected and normalised.
    """
    if _is_gxworks3_export(text):
        text = _normalize_gxworks3(text)

    reader = csv.reader(StringIO(text))
    rows = list(reader)
    if not rows:
        raise CSVParseError("CSV is empty")

    # Detect header row: first cell is not a pure integer.
    if rows and not _STEP_RE.match(rows[0][0].strip() if rows[0] else ''):
        rows = rows[1:]

    instrs: list[Instr] = []
    for i, row in enumerate(rows, 1):
        cells = [c.strip() for c in row]
        # skip wholly-empty rows
        if not any(cells):
            continue

        # expect at minimum: step, instr
        if len(cells) < 2:
            raise CSVParseError(
                f"Row {i}: need at least 2 columns (step, instr); got {row!r}"
            )

        if not _STEP_RE.match(cells[0]):
            raise CSVParseError(
                f"Row {i}: column 1 must be an integer step; got {cells[0]!r}"
            )
        step = int(cells[0])
        op = cells[1].upper()

        # operands: columns 3..N, with column 3 possibly packing multiple
        # tokens separated by whitespace ("T0 K10" pattern).
        args: list[str] = []
        for cell in cells[2:]:
            if cell:
                args.extend(cell.split())

        instrs.append(Instr(step=step, op=op, args=args, line_no=i))

    return instrs


def validate_instructions(instrs: list[Instr]) -> None:
    """Hard-fail on any unknown instruction."""
    for instr in instrs:
        if instr.op not in ALLOWED_INSTRS:
            raise UnknownInstructionError(
                f"Row {instr.line_no} (step {instr.step}): "
                f"instruction {instr.op!r} is outside the supported set "
                f"(Lab 1-15). Supported: {sorted(ALLOWED_INSTRS)}"
            )


# ---------------------------------------------------------------------------
# 4. Operand naming / address allocation
# ---------------------------------------------------------------------------

# Recognised operand prefixes and their semantics.
#   X -- BOOL input,    located at %QX100.<n>     (so monitor can write)
#   Y -- BOOL output,   located at %QX0.<n>
#   M -- BOOL aux flag, located at %QX110.<n>     (memory bit replacement)
#   S -- BOOL state,    located at %QX120.<n>     (only when used as BOOL)
#   T -- TON instance,  unlocated  (FB block)
#   C -- CTU instance,  unlocated  (FB block)
#   D -- INT data reg,  located at %QW101.<n>     (observable from monitor)
# Counter current-value mirrors live at %QW100.<n> (observation only).
# Special relays:
#   SM8000/M8000 -- always TRUE  (constant)
#   SM8002/M8002 -- first-scan pulse  (synthesised)
#   SM8013/M8013 -- 1 Hz square wave  (synthesised via two TONs)

ADDR_BASE = {
    'X': ('%QX100.', 'BOOL'),
    'Y': ('%QX0.',   'BOOL'),
    'M': ('%QX110.', 'BOOL'),
    'S': ('%QX120.', 'BOOL'),
    'D': ('%QW101',  'INT'),     # special: D0->%QW101, D1->%QW102, ...
}

# Counter CV observation slot prefix
CV_ADDR_PREFIX = '%QW100'    # C0.CV -> %QW100, C1.CV -> ... we'll bump


_OPERAND_RE = re.compile(r'^(SM\d+|[A-Z])(\d+)$', re.IGNORECASE)
_KCONST_RE  = re.compile(r'^K(-?\d+)$', re.IGNORECASE)
_DREG_RE    = re.compile(r'^D(\d+)$', re.IGNORECASE)


def _parse_xy_octal(prefix: str, digits: str, orig_token: str) -> int:
    """Parse the digit-part of an X or Y operand as octal per Mitsubishi
    convention (X0..X7, X10..X17, X20..X27, ...; no X8/X9 etc.).

    Returns the decimal bit-index. Range is clamped to 0..15 (X0..X17 octal
    / Y0..Y17 octal), since the il2st address layout reserves bytes 0-1 for
    X and 2-3 for Y in OpenPLC %QX. Out-of-range or non-octal digits are
    rejected.
    """
    # All digits must be in [0-7] -- otherwise the operand label is illegal
    # under Mitsubishi octal numbering (X8, X9, X18, X19 do not exist).
    if not digits or any(c not in '01234567' for c in digits):
        raise IL2STError(
            f"{orig_token!r}: {prefix}{digits} is not a valid Mitsubishi "
            f"address. {prefix} numbering is octal: digits 8 and 9 are not "
            f"used (legal: {prefix}0..{prefix}7, {prefix}10..{prefix}17, ...)."
        )
    bit_idx = int(digits, 8)
    if bit_idx > 15:
        raise IL2STError(
            f"{orig_token!r}: {prefix}{digits} (octal) = bit {bit_idx}, "
            f"which exceeds the supported range. il2st maps {prefix}0..{prefix}17 "
            f"(octal, i.e. 16 points) to a dedicated 2-byte zone in %QX; "
            f"{prefix}20 and above are not supported."
        )
    return bit_idx


def _xy_canonical_name(prefix: str, bit_idx: int) -> str:
    """Inverse of _parse_xy_octal: given a decimal bit-index, return the
    Mitsubishi octal-label as a lower-case ST variable name.

    E.g. bit_idx=0 -> 'x0' (or 'y0'); bit_idx=8 -> 'x10'; bit_idx=15 -> 'x17'.
    """
    return prefix.lower() + oct(bit_idx)[2:]  # oct(8) == '0o10', strip '0o'


def parse_operand(opd: str) -> tuple[str, int | None, str]:
    """
    Parse an operand into (prefix, index, original).

    Returns:
        prefix: 'X','Y','M','S','T','C','D','K','SM','M_SPECIAL'
        index:  for X/Y: DECIMAL bit-index 0..15 (Mitsubishi octal labels are
                normalised here -- X10 -> 8, X17 -> 15);
                for other types: numeric index as written;
                None for K-only.
        original: the original token (used to preserve constants)
    """
    opd_u = opd.upper().strip()
    if not opd_u:
        raise IL2STError("Empty operand")

    # constant
    m = _KCONST_RE.match(opd_u)
    if m:
        return ('K', int(m.group(1)), opd_u)

    # SM-prefixed special relays
    if opd_u.startswith('SM'):
        m = re.match(r'^SM(\d+)$', opd_u)
        if not m:
            raise IL2STError(f"Bad SM operand: {opd!r}")
        return ('SM', int(m.group(1)), opd_u)

    # FX3-style M-special relays we recognise: M8000, M8002, M8013
    m = re.match(r'^M(8000|8002|8013)$', opd_u)
    if m:
        # treat as SM-class
        return ('SM', int(m.group(1)), 'SM' + m.group(1))

    # generic prefix + number
    m = _OPERAND_RE.match(opd_u)
    if m:
        prefix = m.group(1)
        digits = m.group(2)
        if prefix in ('X', 'Y'):
            bit_idx = _parse_xy_octal(prefix, digits, opd_u)
            return (prefix, bit_idx, opd_u)
        if prefix in ('M', 'S', 'T', 'C', 'D'):
            return (prefix, int(digits), opd_u)

    raise IL2STError(f"Unrecognised operand: {opd!r}")


def safe_var_name(name: str) -> str:
    """Return a lower-case version of name, renaming if it collides with a reserved word."""
    low = name.lower()
    if low in RESERVED_WORDS:
        if low in RENAME_MAP:
            return RENAME_MAP[low]
        return low + '_var'
    return low


# ---------------------------------------------------------------------------
# 5. Expression tree (boolean)
# ---------------------------------------------------------------------------

class Expr:
    pass


@dataclass
class Var(Expr):
    name: str
    def __str__(self):
        return self.name


@dataclass
class TimerQ(Expr):
    name: str
    def __str__(self):
        return f"{self.name}.Q"


@dataclass
class CounterQ(Expr):
    name: str
    def __str__(self):
        return f"{self.name}.Q"


@dataclass
class TrigQ(Expr):
    name: str   # already a R_TRIG/F_TRIG instance
    def __str__(self):
        return f"{self.name}.Q"


@dataclass
class Const(Expr):
    val: bool
    def __str__(self):
        return "TRUE" if self.val else "FALSE"


@dataclass
class Not(Expr):
    e: Expr
    def __str__(self):
        return f"(NOT {self.e})"


@dataclass
class And(Expr):
    a: Expr
    b: Expr
    def __str__(self):
        return f"({self.a} AND {self.b})"


@dataclass
class Or(Expr):
    a: Expr
    b: Expr
    def __str__(self):
        return f"({self.a} OR {self.b})"


@dataclass
class Cmp(Expr):
    """Comparison expression for LD=/LD< etc."""
    op: str       # '=', '<>', '<', '>', '<=', '>='
    a: str        # already-rendered term (e.g. 'd0', '5')
    b: str
    def __str__(self):
        # IEC 61131-3 uses '=' for equality, '<>' for inequality.
        return f"({self.a} {self.op} {self.b})"


# ---------------------------------------------------------------------------
# 6. Context -- accumulates everything we need to declare
# ---------------------------------------------------------------------------

@dataclass
class Context:
    program_name: str = 'Prac'
    task_ms: int = 20

    # Variables in use; key is canonical lower name -> metadata dict
    # We index by canonical name to dedupe.
    # ---- Bools (located) ----
    x_used: set = field(default_factory=set)     # set of int indices
    y_used: set = field(default_factory=set)
    m_used: set = field(default_factory=set)
    s_used: set = field(default_factory=set)
    # ---- INT (located) ----
    d_used: set = field(default_factory=set)
    # ---- FB instances ----
    # Tracks which timer/counter indices have already emitted their resolve
    # placeholder. We must NOT use timer_used/counter_used for this, because
    # those also get populated when a timer/counter is READ as a contact
    # (e.g. `AND T1`, `LD C0`) via operand_to_expr -- which can happen BEFORE
    # the `OUT Tn`/`OUT Cn` that should emit the placeholder, causing the
    # placeholder (and thus the whole FB call) to be silently dropped.
    timer_ph_emitted: set = field(default_factory=set)
    counter_ph_emitted: set = field(default_factory=set)
    timer_used: set = field(default_factory=set)   # set of int indices for Tn
    counter_used: set = field(default_factory=set) # for Cn
    rtrig_for_pls: dict = field(default_factory=dict)  # m_index -> rtrig instance name
    ftrig_for_plf: dict = field(default_factory=dict)  # m_index -> ftrig instance name
    # Counter CV mirrors we expose
    cv_used: dict = field(default_factory=dict)    # cn idx -> observation int name

    # Special relays
    use_sm8000: bool = False
    use_sm8002: bool = False
    use_sm8013: bool = False

    # STL data
    stl_blocks: list = field(default_factory=list)  # list of STL block records
    parallel_stl: bool = False                      # True if any block SETs >=2 S

    # variable observation flags (for S used as BOOL in parallel mode)
    s_observation: set = field(default_factory=set)

    # collected RST conditions for each counter (will fold into CTU call)
    counter_reset_conds: dict = field(default_factory=lambda: defaultdict(list))
    # collected timer PT info: tidx -> ('K', kvalue) | ('D', didx)
    timer_pt: dict = field(default_factory=dict)
    # ALL (in_cond, pt_str) pairs for each timer index. A timer used in
    # multiple STL steps with different PTs produces multiple pairs; these get
    # merged into ONE FB call (IN = OR of conds; PT selected by active cond)
    # so the single FB instance is not clobbered by repeated calls.
    timer_calls: dict = field(default_factory=lambda: defaultdict(list))
    # ordering of timer "appearance" so we can emit at first use
    timer_first_use: dict = field(default_factory=dict)
    # likewise for counters
    counter_pv: dict = field(default_factory=dict)
    # ALL (cu_cond, pv) pairs per counter index -- merged into ONE CTU call
    # (CU = OR of conds) so repeated OUT Cn don't clobber the single instance.
    counter_calls: dict = field(default_factory=lambda: defaultdict(list))

    # Tracking for STL decoder pattern:
    # variables (canonical names) ever appearing as OUT target.
    bool_out_targets: set = field(default_factory=set)
    # variables (canonical names) ever appearing as SET/RST target.
    bool_latched: set = field(default_factory=set)
    # D-register indices used as a TIMER PT (e.g. `OUT T0 D0`). These get
    # declared as TIME instead of INT, and any MOV K Dx targeting them is
    # const-folded to a TIME literal (avoids INT_TO_TIME -- whose unit
    # interpretation on OpenPLC v3 MATIEC is unreliable).
    d_pt_used: set = field(default_factory=set)

    # Edge-detect FB pool. Keyed by ('R'|'F', operand_canonical_name); value
    # is the FB instance name (e.g. 'rtrig_x0', 'ftrig_x0'). Used by LDP/LDF/
    # ANDP/ANDF/ORP/ORF so the same operand only allocates one R_TRIG/F_TRIG.
    edge_trigs: dict = field(default_factory=dict)

    # Timer-reset conditions collected from RST Tn. Folded into the timer
    # call's IN so it becomes (cond AND NOT (reset_or...)) which forces reset.
    timer_reset_conds: dict = field(default_factory=lambda: defaultdict(list))

    # Dedicated `tn_pt : TIME` variables created when one timer is used across
    # multiple STL steps with different PT values (PT selected per active step).
    timer_pt_vars: set = field(default_factory=set)

    def canonical_for_operand(self, opd: str) -> str:
        """Return the canonical ST variable name for an operand string."""
        prefix, idx, _ = parse_operand(opd)
        if prefix == 'SM':
            if idx == 8000:
                self.use_sm8000 = True
                return 'TRUE'
            if idx == 8002:
                self.use_sm8002 = True
                return 'sm8002'
            if idx == 8013:
                self.use_sm8013 = True
                return 'sm8013'
            raise IL2STError(f"Unsupported special relay: SM{idx}")
        if prefix == 'X': return safe_var_name(_xy_canonical_name("X", idx))
        if prefix == 'Y': return safe_var_name(_xy_canonical_name("Y", idx))
        if prefix == 'M': return safe_var_name(f"m{idx}")
        if prefix == 'S': return safe_var_name(f"s{idx}")
        if prefix == 'T': return safe_var_name(f"t{idx}")
        if prefix == 'C': return safe_var_name(f"c{idx}")
        if prefix == 'D': return safe_var_name(f"d{idx}")
        raise IL2STError(f"Cannot canonicalize operand: {opd}")


# ---------------------------------------------------------------------------
# 7. Operand -> Expression (records use in ctx as a side-effect)
# ---------------------------------------------------------------------------

def operand_to_expr(opd: str, ctx: Context) -> Expr:
    prefix, idx, _ = parse_operand(opd)

    if prefix == 'SM':
        if idx == 8000:
            ctx.use_sm8000 = True
            return Const(True)
        if idx == 8002:
            ctx.use_sm8002 = True
            return Var('sm8002')
        if idx == 8013:
            ctx.use_sm8013 = True
            return Var('sm8013')
        raise IL2STError(f"Unsupported special relay: SM{idx}")

    if prefix == 'X':
        ctx.x_used.add(idx)
        return Var(safe_var_name(_xy_canonical_name("X", idx)))
    if prefix == 'Y':
        ctx.y_used.add(idx)
        return Var(safe_var_name(_xy_canonical_name("Y", idx)))
    if prefix == 'M':
        ctx.m_used.add(idx)
        return Var(safe_var_name(f"m{idx}"))
    if prefix == 'S':
        ctx.s_used.add(idx)
        return Var(safe_var_name(f"s{idx}"))
    if prefix == 'T':
        ctx.timer_used.add(idx)
        return TimerQ(safe_var_name(f"t{idx}"))
    if prefix == 'C':
        ctx.counter_used.add(idx)
        return CounterQ(safe_var_name(f"c{idx}"))
    if prefix == 'D':
        ctx.d_used.add(idx)
        return Var(safe_var_name(f"d{idx}"))

    raise IL2STError(f"Cannot map operand {opd!r} to expression")


# ---------------------------------------------------------------------------
# 8. Stack simulator
# ---------------------------------------------------------------------------

class StackSim:
    """Mitsubishi stack-machine simulator that builds Expr trees."""

    def __init__(self, ctx: Context):
        self.ctx = ctx
        self.acc: Optional[Expr] = None
        self.stack: list[Expr] = []

    def reset(self):
        self.acc = None
        self.stack = []

    def _start_new_block(self):
        """LD/LDI when acc is active opens a new block, pushing the old acc."""
        if self.acc is not None:
            self.stack.append(self.acc)

    # ---- elementary IL ops ----

    def ld(self, opd):
        self._start_new_block()
        self.acc = operand_to_expr(opd, self.ctx)

    def ldi(self, opd):
        self._start_new_block()
        self.acc = Not(operand_to_expr(opd, self.ctx))

    def and_(self, opd):
        if self.acc is None:
            raise StackError("AND with empty accumulator")
        self.acc = And(self.acc, operand_to_expr(opd, self.ctx))

    def ani(self, opd):
        if self.acc is None:
            raise StackError("ANI with empty accumulator")
        self.acc = And(self.acc, Not(operand_to_expr(opd, self.ctx)))

    def or_(self, opd):
        if self.acc is None:
            raise StackError("OR with empty accumulator")
        self.acc = Or(self.acc, operand_to_expr(opd, self.ctx))

    def ori(self, opd):
        if self.acc is None:
            raise StackError("ORI with empty accumulator")
        self.acc = Or(self.acc, Not(operand_to_expr(opd, self.ctx)))

    def anb(self):
        if not self.stack:
            raise StackError("ANB with empty block stack")
        prev = self.stack.pop()
        self.acc = And(prev, self.acc)

    def orb(self):
        if not self.stack:
            raise StackError("ORB with empty block stack")
        prev = self.stack.pop()
        self.acc = Or(prev, self.acc)

    def mps(self):
        if self.acc is None:
            raise StackError("MPS with empty accumulator")
        self.stack.append(self.acc)

    def mrd(self):
        if not self.stack:
            raise StackError("MRD with empty stack")
        self.acc = self.stack[-1]

    def mpp(self):
        if not self.stack:
            raise StackError("MPP with empty stack")
        self.acc = self.stack.pop()

    # ---- Edge-detection ops (LDP/LDF/ANDP/ANDF/ORP/ORF) ----

    def _edge_expr(self, opd: str, kind: str) -> Expr:
        """Materialise an R_TRIG (kind='R') or F_TRIG (kind='F') on `opd` and
        return an Expr equivalent to its .Q output."""
        # First make sure the operand itself is registered in ctx.
        base_expr = operand_to_expr(opd, self.ctx)
        base_str = str(base_expr)
        # Allocate trig instance if not yet
        key = (kind, base_str)
        if key not in self.ctx.edge_trigs:
            # Generate a safe instance name from base_str.
            safe = re.sub(r'[^a-zA-Z0-9_]', '_', base_str).strip('_').lower()
            prefix = 'rtrig' if kind == 'R' else 'ftrig'
            self.ctx.edge_trigs[key] = f"{prefix}_{safe}"
        trig_name = self.ctx.edge_trigs[key]
        return TrigQ(trig_name)

    def ldp(self, opd):
        self._start_new_block()
        self.acc = self._edge_expr(opd, 'R')

    def ldf(self, opd):
        self._start_new_block()
        self.acc = self._edge_expr(opd, 'F')

    def andp(self, opd):
        if self.acc is None:
            raise StackError("ANDP with empty accumulator")
        self.acc = And(self.acc, self._edge_expr(opd, 'R'))

    def andf(self, opd):
        if self.acc is None:
            raise StackError("ANDF with empty accumulator")
        self.acc = And(self.acc, self._edge_expr(opd, 'F'))

    def orp(self, opd):
        if self.acc is None:
            raise StackError("ORP with empty accumulator")
        self.acc = Or(self.acc, self._edge_expr(opd, 'R'))

    def orf(self, opd):
        if self.acc is None:
            raise StackError("ORF with empty accumulator")
        self.acc = Or(self.acc, self._edge_expr(opd, 'F'))

    # ---- Comparison-contact ops (LD<= D0 K0 etc.) ----

    def _cmp_term(self, t: str) -> str:
        """Convert a comparison operand (K12 or D5) to an ST term string."""
        tu = t.upper()
        if tu.startswith('K'):
            return str(int(tu[1:]))
        if tu.startswith('D'):
            di = int(tu[1:])
            if di in self.ctx.d_pt_used:
                raise IL2STError(
                    f"Comparison cannot use D{di} (declared as TIME)")
            self.ctx.d_used.add(di)
            return safe_var_name(f"d{di}")
        raise IL2STError(f"Comparison operand must be K or D, got {t!r}")

    def ld_cmp(self, op: str, a: str, b: str):
        self._start_new_block()
        self.acc = Cmp(op, self._cmp_term(a), self._cmp_term(b))

    def and_cmp(self, op: str, a: str, b: str):
        if self.acc is None:
            raise StackError("AND<= with empty accumulator")
        self.acc = And(self.acc, Cmp(op, self._cmp_term(a), self._cmp_term(b)))

    def or_cmp(self, op: str, a: str, b: str):
        if self.acc is None:
            raise StackError("OR<= with empty accumulator")
        self.acc = Or(self.acc, Cmp(op, self._cmp_term(a), self._cmp_term(b)))


# ---------------------------------------------------------------------------
# 9. Statement model
# ---------------------------------------------------------------------------

@dataclass
class Statement:
    """One ST statement to emit. Order matters.

    `scope` controls placement during STL emission:
        'arm'    -- stays inside CASE arm / IF s_n THEN block (default)
        'always' -- hoisted outside CASE / IF block (runs every scan)
        'decode' -- a Y/M OUTPUT inside an STL block; the emit phase fuses
                    multiple `decode` statements per target into a single
                    OR-of-conditions assignment outside the CASE.
    """
    kind: str
    text: str
    scope: str = 'arm'
    # for 'decode' statements only
    target_var: Optional[str] = None
    cond_expr: Optional[Expr] = None


# ---------------------------------------------------------------------------
# 10. K -> TIME literal
# ---------------------------------------------------------------------------

def k_to_time_literal(k: int) -> str:
    """Convert FX5U K count (in 100 ms units) to an IEC TIME literal."""
    ms = k * K_PER_TICK_MS
    if ms == 0:
        return "T#0ms"
    # Use ms throughout for simplicity & precision
    return f"T#{ms}ms"


# ---------------------------------------------------------------------------
# 11. STL block pre-scan
# ---------------------------------------------------------------------------

@dataclass
class STLBlock:
    state_index: int       # the n in STL Sn (or None for the "outside" block)
    instrs: list           # the instructions within this STL block
    sets_in_block: list    # list of indices that this block SETs


def pre_scan_stl(instrs: list[Instr], ctx: Context) -> tuple[list, list]:
    """
    Split instructions into pre-STL (before any STL) and STL blocks.

    Returns:
        pre_stl_instrs: instructions before the first STL (and outside)
        stl_blocks: list of STLBlock

    Side effects on ctx:
        ctx.parallel_stl    -- True if any block SETs two or more Sn
        ctx.d_pt_used       -- D indices used as timer PT (OUT Tn Dx pattern)
    """
    pre_stl: list[Instr] = []
    blocks: list[STLBlock] = []
    current: Optional[STLBlock] = None

    for instr in instrs:
        # Scan for variable-PT timers, regardless of STL nesting.
        if instr.op == 'OUT' and len(instr.args) >= 2:
            try:
                pfx, idx, _ = parse_operand(instr.args[0])
            except Exception:
                pfx = None
            if pfx == 'T' and instr.args[1].upper().startswith('D'):
                ctx.d_pt_used.add(int(instr.args[1][1:]))

        if instr.op == 'STL':
            if not instr.args:
                raise IL2STError(f"Row {instr.line_no}: STL with no operand")
            _, sidx, _ = parse_operand(instr.args[0])
            current = STLBlock(state_index=sidx, instrs=[], sets_in_block=[])
            blocks.append(current)
            continue
        if instr.op == 'RET':
            current = None
            continue
        if current is None:
            pre_stl.append(instr)
        else:
            current.instrs.append(instr)
            if instr.op == 'SET' and instr.args:
                try:
                    pfx, sidx, _ = parse_operand(instr.args[0])
                except Exception:
                    continue
                if pfx == 'S':
                    current.sets_in_block.append(sidx)

    # parallel divergence detection: any block with >=2 distinct SET S
    for b in blocks:
        if len(set(b.sets_in_block)) >= 2:
            ctx.parallel_stl = True
            break

    # Pre-populate s_used with EVERY S that is an STL state or a SET/RST/OUT
    # target. This must happen before translation: a `ZRST S_lo S_hi` (e.g. an
    # X0 emergency-stop in the pre-STL section) is translated early, and to
    # expand correctly it needs to know the full set of S states -- otherwise
    # it would only clear the states translated so far (just s0), silently
    # failing to reset the rest of the state machine.
    for instr in instrs:
        if instr.op in ('STL', 'SET', 'RST', 'OUT') and instr.args:
            try:
                pfx, sidx, _ = parse_operand(instr.args[0])
            except Exception:
                continue
            if pfx == 'S':
                ctx.s_used.add(sidx)

    return pre_stl, blocks


# ---------------------------------------------------------------------------
# 12. Translator core
# ---------------------------------------------------------------------------

def translate_pre_stl(instrs: list[Instr], ctx: Context) -> list[Statement]:
    """Translate the non-STL portion of the program."""
    return _translate_linear(instrs, ctx, stl_state_name=None, in_case=False)


def _translate_linear(instrs: list[Instr], ctx: Context,
                      stl_state_name: Optional[str],
                      in_case: bool,
                      initial_acc: Optional[Expr] = None) -> list[Statement]:
    """
    Translate a sequential block of IL into ST Statements.

    Args:
        stl_state_name: if inside an STL block:
                        * for CASE mode: the value to compare (e.g. 'state = 1')
                        * for BOOL mode: the S variable name (e.g. 's21')
                        (Only used for SET-S transitions, since OUT/SET/RST on
                         Y/M etc. inside an STL block naturally include S in the
                         accumulator because the block starts with LD Sn.)
        in_case: True for the CASE/state-machine emission style.
        initial_acc: optional starting accumulator value, used to seed
                     STL blocks with their Sn condition.
    """
    out: list[Statement] = []
    sim = StackSim(ctx)
    if initial_acc is not None:
        sim.acc = initial_acc

    for instr in instrs:
        op = instr.op
        args = instr.args

        try:
            if op == 'END':
                break

            elif op == 'LD':
                _require_args(instr, 1)
                sim.ld(args[0])
            elif op == 'LDI':
                _require_args(instr, 1)
                sim.ldi(args[0])
            elif op == 'AND':
                _require_args(instr, 1)
                sim.and_(args[0])
            elif op == 'ANI':
                _require_args(instr, 1)
                sim.ani(args[0])
            elif op == 'OR':
                _require_args(instr, 1)
                sim.or_(args[0])
            elif op == 'ORI':
                _require_args(instr, 1)
                sim.ori(args[0])
            elif op == 'LDP':
                _require_args(instr, 1)
                sim.ldp(args[0])
            elif op == 'LDF':
                _require_args(instr, 1)
                sim.ldf(args[0])
            elif op == 'ANDP':
                _require_args(instr, 1)
                sim.andp(args[0])
            elif op == 'ANDF':
                _require_args(instr, 1)
                sim.andf(args[0])
            elif op == 'ORP':
                _require_args(instr, 1)
                sim.orp(args[0])
            elif op == 'ORF':
                _require_args(instr, 1)
                sim.orf(args[0])
            elif _split_cmp_op(op) is not None:
                base, cmp = _split_cmp_op(op)
                _require_args(instr, 2)
                if base == 'LD':
                    sim.ld_cmp(cmp, args[0], args[1])
                elif base == 'AND':
                    sim.and_cmp(cmp, args[0], args[1])
                elif base == 'OR':
                    sim.or_cmp(cmp, args[0], args[1])
            elif op == 'ANB':
                sim.anb()
            elif op == 'ORB':
                sim.orb()
            elif op == 'MPS':
                sim.mps()
            elif op == 'MRD':
                sim.mrd()
            elif op == 'MPP':
                sim.mpp()

            elif op == 'OUT':
                _emit_out(instr, sim, ctx, out)

            elif op == 'SET':
                _emit_set(instr, sim, ctx, out, stl_state_name, in_case)

            elif op == 'RST':
                _emit_rst(instr, sim, ctx, out)

            elif op == 'PLS':
                _emit_pls(instr, sim, ctx, out)

            elif op == 'PLF':
                _emit_plf(instr, sim, ctx, out)

            elif op == 'MOV':
                _emit_mov(instr, sim, ctx, out)

            elif op in ('INC', 'INCP'):
                _emit_incdec(instr, sim, ctx, out, +1, pulse=(op == 'INCP'))

            elif op in ('DEC', 'DECP'):
                _emit_incdec(instr, sim, ctx, out, -1, pulse=(op == 'DECP'))

            elif op == 'ZRST':
                _emit_zrst(instr, sim, ctx, out)

            elif op == 'CMP':
                _emit_cmp(instr, sim, ctx, out)

            else:
                raise UnknownInstructionError(f"Unhandled op {op}")

        except IL2STError as e:
            raise IL2STError(
                f"Row {instr.line_no} (step {instr.step}, op {op}): {e}"
            ) from e

    return out


def _require_args(instr: Instr, n: int):
    if len(instr.args) < n:
        raise IL2STError(f"{instr.op} requires {n} operand(s), got {instr.args}")


def _cond_str(sim: StackSim) -> str:
    """Get the current accumulator condition as an ST expression string."""
    if sim.acc is None:
        return "TRUE"
    return str(sim.acc)


# ---- OUT handler -----------------------------------------------------------

def _emit_out(instr: Instr, sim: StackSim, ctx: Context, out: list[Statement]):
    if not instr.args:
        raise IL2STError("OUT requires at least 1 operand")

    target = instr.args[0]
    prefix, idx, _ = parse_operand(target)
    cond = _cond_str(sim)
    cond_expr = sim.acc if sim.acc is not None else Const(True)

    if prefix == 'Y':
        ctx.y_used.add(idx)
        name = safe_var_name(_xy_canonical_name("Y", idx))
        # `decode`: in STL emission, multiple decode lines per target are
        # OR-fused into a single assignment outside the CASE / IF block.
        # In non-STL context the emit phase still renders this as a normal
        # assignment; `target_var` and `cond_expr` carry the info.
        out.append(Statement('ASSIGN', f"{name} := {cond};",
                             scope='decode',
                             target_var=name,
                             cond_expr=cond_expr))
        ctx.bool_out_targets.add(name)

    elif prefix == 'M':
        ctx.m_used.add(idx)
        name = safe_var_name(f"m{idx}")
        out.append(Statement('ASSIGN', f"{name} := {cond};",
                             scope='decode',
                             target_var=name,
                             cond_expr=cond_expr))
        ctx.bool_out_targets.add(name)

    elif prefix == 'T':
        # OUT Tn Kxx | OUT Tn Dxx. A timer may be used in multiple STL steps
        # with different PTs; we collect every (cond, pt) pair and emit ONE
        # merged FB call later so the single instance isn't clobbered.
        if len(instr.args) < 2:
            raise IL2STError(f"OUT Tn requires a K or D operand (got: {instr.args})")
        first_time = idx not in ctx.timer_ph_emitted
        ctx.timer_used.add(idx)
        pt_arg = instr.args[1]
        if pt_arg.upper().startswith('K'):
            k = int(pt_arg[1:])
            pt_str = k_to_time_literal(k)
        elif pt_arg.upper().startswith('D'):
            didx = int(pt_arg[1:])
            pt_str = safe_var_name(f"d{didx}")
        else:
            raise IL2STError(f"Unrecognised timer PT operand: {pt_arg}")
        ctx.timer_calls[idx].append((cond, pt_str))
        # Emit the placeholder only on first appearance; assemble resolves it
        # using the full timer_calls[idx] list.
        if first_time:
            ctx.timer_ph_emitted.add(idx)
            ctx.timer_first_use.setdefault(idx, cond)
            out.append(Statement('TIMER', f"__TON__{idx}__",
                                 scope='always'))

    elif prefix == 'C':
        # OUT Cn Kxx. Collect every (cu_cond, pv) pair; merge into one CTU
        # call later so repeated uses of the same counter aren't clobbered.
        if len(instr.args) < 2:
            raise IL2STError(f"OUT Cn requires K operand (got: {instr.args})")
        first_time = idx not in ctx.counter_ph_emitted
        ctx.counter_used.add(idx)
        pv_arg = instr.args[1]
        if not pv_arg.upper().startswith('K'):
            raise IL2STError(f"OUT Cn PV must be Kxx, got {pv_arg}")
        pv = int(pv_arg[1:])
        ctx.counter_pv[idx] = pv
        ctx.counter_calls[idx].append((cond, pv))
        if first_time:
            ctx.counter_ph_emitted.add(idx)
            out.append(Statement('COUNTER', f"__CTU__{idx}__",
                                 scope='always'))

    elif prefix == 'S':
        ctx.s_used.add(idx)
        name = safe_var_name(f"s{idx}")
        out.append(Statement('ASSIGN', f"{name} := {cond};"))

    else:
        raise IL2STError(f"OUT target {target!r} not supported (prefix {prefix})")


# ---- SET / RST handlers ----------------------------------------------------

def _emit_set(instr: Instr, sim: StackSim, ctx: Context,
              out: list[Statement], stl_state_name: Optional[str], in_case: bool):
    target = instr.args[0]
    prefix, idx, _ = parse_operand(target)
    cond = _cond_str(sim)

    if prefix == 'S':
        # State transition. Marked specially so STL emission can patch it.
        ctx.s_used.add(idx)
        out.append(Statement('STL_TRANS', f"__SET_S__{idx}__ COND={cond}",
                             scope='arm'))
        return

    if prefix == 'Y':
        ctx.y_used.add(idx)
        name = safe_var_name(_xy_canonical_name("Y", idx))
    elif prefix == 'M':
        ctx.m_used.add(idx)
        name = safe_var_name(f"m{idx}")
    else:
        raise IL2STError(f"SET on {target!r} not supported")

    ctx.bool_latched.add(name)
    # Latch pattern: IF cond THEN var := TRUE; END_IF;
    out.append(Statement('IF_TRUE',
                         f"IF {cond} THEN\n  {name} := TRUE;\nEND_IF;",
                         scope='arm'))


def _emit_rst(instr: Instr, sim: StackSim, ctx: Context, out: list[Statement]):
    target = instr.args[0]
    prefix, idx, _ = parse_operand(target)
    cond = _cond_str(sim)

    if prefix == 'C':
        # Reset on counter -- fold into the CTU call.
        ctx.counter_used.add(idx)
        ctx.counter_reset_conds[idx].append(cond)
        out.append(Statement('COUNTER_RST', f"__CTU_RST__{idx}__ COND={cond}",
                             scope='always'))
        return

    if prefix == 'T':
        # Reset on timer -- TON has no R input, so we fold the reset into the
        # IN expression of the matching OUT Tn call later (post-process).
        ctx.timer_used.add(idx)
        ctx.timer_reset_conds[idx].append(cond)
        out.append(Statement('TIMER_RST', f"__TON_RST__{idx}__ COND={cond}",
                             scope='always'))
        return

    if prefix == 'S':
        # RST Sn -- self-kill (especially in parallel mode)
        ctx.s_used.add(idx)
        out.append(Statement('STL_TRANS',
                             f"__RST_S__{idx}__ COND={cond}",
                             scope='arm'))
        return

    if prefix == 'Y':
        ctx.y_used.add(idx)
        name = safe_var_name(_xy_canonical_name("Y", idx))
    elif prefix == 'M':
        ctx.m_used.add(idx)
        name = safe_var_name(f"m{idx}")
    else:
        raise IL2STError(f"RST on {target!r} not supported")

    ctx.bool_latched.add(name)
    out.append(Statement('IF_FALSE',
                         f"IF {cond} THEN\n  {name} := FALSE;\nEND_IF;",
                         scope='arm'))


# ---- PLS / PLF handlers ----------------------------------------------------

def _emit_pls(instr: Instr, sim: StackSim, ctx: Context, out: list[Statement]):
    target = instr.args[0]
    prefix, idx, _ = parse_operand(target)
    if prefix != 'M':
        raise IL2STError(f"PLS target must be M, got {target}")
    ctx.m_used.add(idx)
    cond = _cond_str(sim)
    trig_name = f"rtrig_m{idx}"
    m_name = safe_var_name(f"m{idx}")
    ctx.rtrig_for_pls[idx] = trig_name
    ctx.bool_latched.add(m_name)   # treat as latching-type (driven by FB)
    out.append(Statement('TRIG',
                         f"{trig_name}(CLK := {cond});\n{m_name} := {trig_name}.Q;",
                         scope='always'))


def _emit_plf(instr: Instr, sim: StackSim, ctx: Context, out: list[Statement]):
    target = instr.args[0]
    prefix, idx, _ = parse_operand(target)
    if prefix != 'M':
        raise IL2STError(f"PLF target must be M, got {target}")
    ctx.m_used.add(idx)
    cond = _cond_str(sim)
    trig_name = f"ftrig_m{idx}"
    m_name = safe_var_name(f"m{idx}")
    ctx.ftrig_for_plf[idx] = trig_name
    ctx.bool_latched.add(m_name)
    out.append(Statement('TRIG',
                         f"{trig_name}(CLK := {cond});\n{m_name} := {trig_name}.Q;",
                         scope='always'))


# ---- Data-instruction handlers --------------------------------------------

def _emit_mov(instr: Instr, sim: StackSim, ctx: Context, out: list[Statement]):
    # MOV <src> <dst> ; src is Kxx or Dxx, dst is Dxx
    if len(instr.args) < 2:
        raise IL2STError(f"MOV requires 2 operands (src, dst), got {instr.args}")
    src_raw, dst_raw = instr.args[0], instr.args[1]
    if not dst_raw.upper().startswith('D'):
        raise IL2STError(f"MOV destination must be D, got {dst_raw}")
    didx = int(dst_raw[1:])
    dst = safe_var_name(f"d{didx}")
    cond = _cond_str(sim)

    # ---- TIME-typed destination (D used as timer PT) ----
    if didx in ctx.d_pt_used:
        if src_raw.upper().startswith('K'):
            # const-fold to TIME literal: K count * 100 ms (FX5U tick)
            k = int(src_raw[1:])
            ms = k * K_PER_TICK_MS
            time_lit = f"T#{ms}ms"
            out.append(Statement('MOV',
                                 f"IF {cond} THEN\n  {dst} := {time_lit};\nEND_IF;"))
            return
        if src_raw.upper().startswith('D'):
            si = int(src_raw[1:])
            if si not in ctx.d_pt_used:
                raise IL2STError(
                    f"MOV D{si} -> D{didx}: source must also be a TIME-typed D "
                    f"(timer PT), but D{si} is INT. Consider rewriting the IL.")
            sname = safe_var_name(f"d{si}")
            out.append(Statement('MOV',
                                 f"IF {cond} THEN\n  {dst} := {sname};\nEND_IF;"))
            return
        raise IL2STError(f"MOV source must be K or D, got {src_raw}")

    # ---- INT-typed destination (regular data register) ----
    ctx.d_used.add(didx)
    if src_raw.upper().startswith('K'):
        src = str(int(src_raw[1:]))
    elif src_raw.upper().startswith('D'):
        si = int(src_raw[1:])
        if si in ctx.d_pt_used:
            raise IL2STError(
                f"MOV D{si} (TIME) -> D{didx} (INT): cannot mix types")
        ctx.d_used.add(si)
        src = safe_var_name(f"d{si}")
    else:
        raise IL2STError(f"MOV source must be K or D, got {src_raw}")
    out.append(Statement('MOV',
                         f"IF {cond} THEN\n  {dst} := {src};\nEND_IF;"))


def _emit_incdec(instr: Instr, sim: StackSim, ctx: Context,
                 out: list[Statement], delta: int, pulse: bool):
    """INC / INCP / DEC / DECP -- increment or decrement a D register.

    `delta` is +1 (INC*) or -1 (DEC*); `pulse` toggles edge-triggered.
    """
    if not instr.args:
        raise IL2STError("INC/DEC requires 1 operand")
    target = instr.args[0]
    if not target.upper().startswith('D'):
        raise IL2STError(f"INC/DEC target must be D, got {target}")
    didx = int(target[1:])
    if didx in ctx.d_pt_used:
        raise IL2STError(
            f"INC/DEC D{didx}: D{didx} is used as a timer PT (TIME-typed); "
            f"incrementing TIME is not supported.")
    ctx.d_used.add(didx)
    dname = safe_var_name(f"d{didx}")
    cond = _cond_str(sim)
    op_label = ('INCP' if delta > 0 else 'DECP') if pulse else \
               ('INC' if delta > 0 else 'DEC')
    sign = '+' if delta > 0 else '-'
    if pulse:
        trig_name = f"rtrig_{op_label.lower()}_d{didx}"
        ctx.rtrig_for_pls[1000 + (didx if delta > 0 else -didx - 1)] = trig_name
        out.append(Statement(op_label,
                             f"{trig_name}(CLK := {cond});\n"
                             f"IF {trig_name}.Q THEN\n"
                             f"  {dname} := {dname} {sign} 1;\nEND_IF;"))
    else:
        # Every-scan: while cond holds, accumulate. This matches Mitsubishi
        # INC/DEC semantics (which add per scan).
        out.append(Statement(op_label,
                             f"IF {cond} THEN\n"
                             f"  {dname} := {dname} {sign} 1;\nEND_IF;"))


def _emit_zrst(instr: Instr, sim: StackSim, ctx: Context, out: list[Statement]):
    """ZRST <lo> <hi> -- zero a contiguous range. Supports D / Y / M / S."""
    if len(instr.args) < 2:
        raise IL2STError("ZRST requires 2 operands")
    a, b = instr.args[0], instr.args[1]
    a_pfx = a[0].upper()
    b_pfx = b[0].upper()
    if a_pfx != b_pfx:
        raise IL2STError(f"ZRST endpoints must share prefix, got {a},{b}")
    if a_pfx not in ('D', 'Y', 'M', 'S'):
        raise IL2STError(f"ZRST only supports D/Y/M/S, got {a_pfx}")
    # Y endpoints are octal per Mitsubishi convention; D/M/S are decimal.
    if a_pfx == 'Y':
        lo = _parse_xy_octal('Y', a[1:], a)
        hi = _parse_xy_octal('Y', b[1:], b)
    else:
        lo = int(a[1:])
        hi = int(b[1:])
    if hi < lo:
        lo, hi = hi, lo
    cond = _cond_str(sim)

    if a_pfx == 'D':
        lines = [f"IF {cond} THEN"]
        for di in range(lo, hi + 1):
            dn = safe_var_name(f"d{di}")
            if di in ctx.d_pt_used:
                lines.append(f"  {dn} := T#0ms;")
            else:
                ctx.d_used.add(di)
                lines.append(f"  {dn} := 0;")
        lines.append("END_IF;")
        out.append(Statement('ZRST', "\n".join(lines)))
        return

    if a_pfx == 'Y':
        # Emit one-line FALSE assignments for every Y in range. Add them all
        # to y_used so they get declared even if no other instruction touched
        # them. (Typical use: emergency ZRST Y0 Y17 wipes the bank.)
        lines = [f"IF {cond} THEN"]
        for yi in range(lo, hi + 1):
            ctx.y_used.add(yi)
            yn = safe_var_name(_xy_canonical_name("Y", yi))
            lines.append(f"  {yn} := FALSE;")
        lines.append("END_IF;")
        out.append(Statement('ZRST', "\n".join(lines)))
        return

    if a_pfx == 'M':
        lines = [f"IF {cond} THEN"]
        for mi in range(lo, hi + 1):
            ctx.m_used.add(mi)
            mn = safe_var_name(f"m{mi}")
            lines.append(f"  {mn} := FALSE;")
        lines.append("END_IF;")
        out.append(Statement('ZRST', "\n".join(lines)))
        return

    if a_pfx == 'S':
        # S-zone reset. In linear STL mode, the single state INT subsumes all
        # S-flags, so emitting `state := 0` cleanly stops the machine. In
        # parallel STL mode each S is a BOOL: only clear the ones we actually
        # declared (others would be undeclared-var errors).
        if ctx.parallel_stl:
            lines = [f"IF {cond} THEN"]
            for si in range(lo, hi + 1):
                if si in ctx.s_used:
                    sn = safe_var_name(f"s{si}")
                    lines.append(f"  {sn} := FALSE;")
            lines.append("END_IF;")
            # If nothing matched, emit an empty no-op statement (still
            # valid ST). Otherwise emit the IF.
            if len(lines) == 2:
                lines = ["(* ZRST S-range with no declared S; no-op *)"]
            out.append(Statement('ZRST', "\n".join(lines)))
        else:
            out.append(Statement('ZRST',
                                 f"IF {cond} THEN\n  state := 0;\nEND_IF;"))
        return


def _emit_cmp(instr: Instr, sim: StackSim, ctx: Context, out: list[Statement]):
    # CMP <a> <b> <M_base>
    if len(instr.args) < 3:
        raise IL2STError("CMP requires 3 operands (s1, s2, dst M)")
    a_raw, b_raw, m_raw = instr.args[:3]

    def to_term(t):
        if t.upper().startswith('K'):
            return str(int(t[1:]))
        if t.upper().startswith('D'):
            di = int(t[1:])
            if di in ctx.d_pt_used:
                raise IL2STError(
                    f"CMP cannot compare D{di} (TIME-typed) as INT")
            ctx.d_used.add(di)
            return safe_var_name(f"d{di}")
        raise IL2STError(f"CMP operand must be K or D, got {t}")

    aexp = to_term(a_raw)
    bexp = to_term(b_raw)
    if not m_raw.upper().startswith('M'):
        raise IL2STError("CMP dest must be Mn")
    mbase = int(m_raw[1:])
    ctx.m_used.add(mbase)
    ctx.m_used.add(mbase + 1)
    ctx.m_used.add(mbase + 2)
    m_gt = safe_var_name(f"m{mbase}")
    m_eq = safe_var_name(f"m{mbase+1}")
    m_lt = safe_var_name(f"m{mbase+2}")
    cond = _cond_str(sim)
    lines = [f"IF {cond} THEN",
             f"  {m_gt} := {aexp} > {bexp};",
             f"  {m_eq} := {aexp} = {bexp};",
             f"  {m_lt} := {aexp} < {bexp};",
             "END_IF;"]
    out.append(Statement('CMP', "\n".join(lines)))


# ---------------------------------------------------------------------------
# 13. STL translation
# ---------------------------------------------------------------------------

def translate_stl(blocks: list[STLBlock], ctx: Context) -> list[Statement]:
    """
    Translate STL blocks to ST.

    Two modes:
      * Linear  -- state : INT + CASE statement.
      * Parallel -- one BOOL per S; each block lives in its own IF s_n THEN ...
    """
    if not blocks:
        return []

    if ctx.parallel_stl:
        return _translate_stl_parallel(blocks, ctx)
    else:
        return _translate_stl_linear(blocks, ctx)


def _translate_stl_linear(blocks: list[STLBlock], ctx: Context) -> list[Statement]:
    """Linear STL -> `state : INT` driven by CASE, with decoder pattern for OUTs.

    Statement scope routing:
      * 'always' (TON/CTU/PLS/PLF/RST C) -- emitted OUTSIDE CASE (every scan)
      * 'decode' (OUT Y/OUT M)           -- collected per-target, OR-fused,
                                            emitted OUTSIDE CASE as one assign
      * 'arm'    (SET/RST Y,M / state-trans / MOV / CMP / etc) -- inside CASE
    """

    state_order: list[int] = []
    for b in blocks:
        if b.state_index not in state_order:
            state_order.append(b.state_index)
    state_to_idx = {s: i + 1 for i, s in enumerate(state_order)}
    ctx._linear_state_map = state_to_idx
    ctx._linear_state_used = True

    out: list[Statement] = []

    # ----- 1. State decoder: s_n := (state = N) -----
    for sidx, n in state_to_idx.items():
        ctx.s_used.add(sidx)
        sname = safe_var_name(f"s{sidx}")
        out.append(Statement('ASSIGN', f"{sname} := (state = {n});"))

    # ----- 2. Translate each block with s_n seed; collect statements -----
    decode_collected: OrderedDict = OrderedDict()   # target_var -> list[cond_str]
    always_stmts: list[Statement] = []              # in encounter order
    arm_stmts: list[tuple[int, list[Statement]]] = []  # (state_idx_label, [stmts])

    for b in blocks:
        ctx.s_used.add(b.state_index)
        sname = safe_var_name(f"s{b.state_index}")
        seed = Var(sname)
        raw = _translate_linear(b.instrs, ctx,
                                stl_state_name=None,
                                in_case=True,
                                initial_acc=seed)
        # patch placeholders
        patched = []
        for s in raw:
            ps = _postprocess_stmt(s, ctx, state_to_idx)
            if ps is not None:
                patched.append(ps)

        block_arm: list[Statement] = []
        for s in patched:
            if s.scope == 'decode':
                # collect, do not emit in arm
                decode_collected.setdefault(s.target_var, []).append(
                    str(s.cond_expr))
            elif s.scope == 'always':
                always_stmts.append(s)
            else:
                # 'arm' (default) -- stays inside CASE
                block_arm.append(s)
        arm_stmts.append((state_to_idx[b.state_index], block_arm))

    # ----- 3. Emit decoder lines (OUT Y/M -> single OR-fused assignment) -----
    for target, conds in decode_collected.items():
        if len(conds) == 1:
            expr = conds[0]
        else:
            expr = " OR ".join(f"({c})" for c in conds)
        out.append(Statement('DECODE', f"{target} := {expr};"))

    # ----- 4. Emit 'always' statements (timers, counters, edge triggers) -----
    for s in always_stmts:
        out.append(s)

    # ----- 5. Emit CASE (only state transitions, SET/RST Y/M, MOV/CMP/etc) -----
    case_lines = ["CASE state OF"]
    any_arm_body = False
    for n, stmts in arm_stmts:
        if not stmts:
            continue
        any_arm_body = True
        case_lines.append(f"  {n}:")
        for s in stmts:
            for ln in s.text.splitlines():
                case_lines.append(f"    {ln}")
    case_lines.append("END_CASE;")
    if any_arm_body:
        out.append(Statement('CASE', "\n".join(case_lines)))

    return out


def _translate_stl_parallel(blocks: list[STLBlock], ctx: Context) -> list[Statement]:
    """Parallel STL -> one BOOL per S; IF s_n THEN ... END_IF; per block.

    Routing parallels the linear-mode treatment:
      * decode: OUT Y/M  -> OR-fused single assignment outside any IF s_n
      * always: TON/CTU/PLS/PLF  -> outside any IF s_n (every scan)
      * arm:    SET/RST/state-trans/MOV/CMP  -> inside the IF s_n

    Mitsubishi STL semantics: SET S_target in an STL block also implicitly
    deactivates the current state. To support parallel divergence (multiple
    SETs from one block must all fire on the same scan), we accumulate
    transition conditions and emit a single self-deactivation at the end
    of the IF block, triggered by the OR of all transitions.
    """
    out: list[Statement] = []
    decode_collected: OrderedDict = OrderedDict()
    always_stmts: list[Statement] = []
    if_blocks: list[tuple[str, list[Statement], list[str]]] = []
    # ^ (self_var, body_stmts, transition_conds)

    for b in blocks:
        ctx.s_used.add(b.state_index)
        s_self = safe_var_name(f"s{b.state_index}")
        seed = Var(s_self)
        raw = _translate_linear(b.instrs, ctx,
                                stl_state_name=s_self,
                                in_case=False,
                                initial_acc=seed)

        transition_conds: list[str] = []
        patched: list[Statement] = []
        for s in raw:
            # Capture SET S transitions for delayed self-reset.
            m = re.match(r'^__SET_S__(\d+)__ COND=(.+)$', s.text)
            if m:
                sidx = int(m.group(1))
                cond = m.group(2)
                s_target = safe_var_name(f"s{sidx}")
                if s_target != s_self:
                    transition_conds.append(cond)
                # Write to the shadow var (s_target_next) so a SET S inside
                # this scan does not cause the IF s_target THEN block (which
                # may run later in the same scan) to fire on the just-set
                # state -- that would cascade Sa -> Sb -> Sc -> ... in one
                # scan, violating "one step per scan" STL semantics.
                patched.append(Statement(
                    'STL_TRANS',
                    f"IF {cond} THEN\n  {s_target}_next := TRUE;\nEND_IF;",
                    scope='arm'))
                continue
            # RST S placeholder.
            m = re.match(r'^__RST_S__(\d+)__ COND=(.+)$', s.text)
            if m:
                sidx = int(m.group(1))
                cond = m.group(2)
                s_target = safe_var_name(f"s{sidx}")
                if s_target == s_self:
                    # Self-RST: defer to block end. In Mitsubishi STL,
                    # resetting the current step only takes effect on the
                    # NEXT scan -- the remaining instructions in THIS block
                    # (e.g. a following ZRST Y0 Y3) must still execute this
                    # scan. Emitting `s_self := FALSE` inline here would gate
                    # them out, because their own IF s_self condition would
                    # already be false. So we route it through the same
                    # delayed self-deactivation as state transitions.
                    transition_conds.append(cond)
                else:
                    # RST of a different state: emit immediately on the shadow.
                    patched.append(Statement(
                        'STL_TRANS',
                        f"IF {cond} THEN\n  {s_target}_next := FALSE;\nEND_IF;",
                        scope='arm'))
                continue
            # Otherwise: generic postprocessor (CTU folding, etc.).
            ps = _postprocess_stmt(s, ctx, state_map=None)
            if ps is not None:
                patched.append(ps)

        arm_body: list[Statement] = []
        for s in patched:
            if s.scope == 'decode':
                decode_collected.setdefault(s.target_var, []).append(
                    str(s.cond_expr))
            elif s.scope == 'always':
                always_stmts.append(s)
            else:
                arm_body.append(s)
        if_blocks.append((s_self, arm_body, transition_conds))

    # decoder outputs
    for target, conds in decode_collected.items():
        if len(conds) == 1:
            expr = conds[0]
        else:
            expr = " OR ".join(f"({c})" for c in conds)
        out.append(Statement('DECODE', f"{target} := {expr};"))

    for s in always_stmts:
        out.append(s)

    for s_self, body, t_conds in if_blocks:
        if not body and not t_conds:
            continue
        block_lines = [f"IF {s_self} THEN"]
        for stmt in body:
            for ln in stmt.text.splitlines():
                block_lines.append(f"  {ln}")
        # consolidated self-deactivation
        if t_conds:
            # dedupe identical conditions
            uniq: list[str] = []
            for c in t_conds:
                if c not in uniq:
                    uniq.append(c)
            if len(uniq) == 1:
                trans_or = uniq[0]
            else:
                trans_or = " OR ".join(f"({c})" for c in uniq)
            block_lines.append(f"  IF {trans_or} THEN")
            block_lines.append(f"    {s_self}_next := FALSE;")
            block_lines.append("  END_IF;")
        block_lines.append("END_IF;")
        out.append(Statement('STL_BLOCK', "\n".join(block_lines)))

    return out


def _postprocess_stl_trans_parallel(stmt: Statement, ctx: Context,
                                    s_self: str) -> Optional[Statement]:
    """Unused now; kept as a stub in case future modes want it."""
    return None


def _postprocess_stmt(stmt: Statement, ctx: Context,
                      state_map: Optional[dict]) -> Optional[Statement]:
    """
    Replace placeholder texts inserted earlier (STL_TRANS, COUNTER, COUNTER_RST).

    Returns None if the statement should be dropped (e.g. RST C placeholders
    that have been folded into the CTU call).
    """
    t = stmt.text

    # STL transitions
    m = re.match(r'^__SET_S__(\d+)__ COND=(.+)$', t)
    if m:
        sidx = int(m.group(1))
        cond = m.group(2)
        if state_map is not None and sidx in state_map:
            return Statement('STL_TRANS',
                             f"IF {cond} THEN\n  state := {state_map[sidx]};\nEND_IF;",
                             scope='arm')
        sname = safe_var_name(f"s{sidx}")
        # Parallel mode: write to the per-scan shadow so a transition triggered
        # in pre-STL or one IF s_n block doesn't cascade into another IF s_n
        # in the same scan (the STL "one step per scan" invariant).
        if ctx.parallel_stl:
            sname = f"{sname}_next"
        return Statement('STL_TRANS',
                         f"IF {cond} THEN\n  {sname} := TRUE;\nEND_IF;",
                         scope='arm')

    m = re.match(r'^__RST_S__(\d+)__ COND=(.+)$', t)
    if m:
        sidx = int(m.group(1))
        cond = m.group(2)
        if state_map is not None and sidx in state_map:
            return Statement('STL_TRANS',
                             f"IF {cond} THEN\n  state := 0;\nEND_IF;",
                             scope='arm')
        sname = safe_var_name(f"s{sidx}")
        if ctx.parallel_stl:
            sname = f"{sname}_next"
        return Statement('STL_TRANS',
                         f"IF {cond} THEN\n  {sname} := FALSE;\nEND_IF;",
                         scope='arm')

    m = re.match(r'^__CTU__(\d+)__$', t)
    if m:
        # Keep placeholder; resolve in assemble after all blocks visited.
        return stmt

    if t.startswith('__CTU_RST__'):
        return None

    m = re.match(r'^__TON__(\d+)__$', t)
    if m:
        # Keep the placeholder intact here. Resolution is deferred to assemble
        # (after ALL STL blocks are translated), because a timer used across
        # multiple steps only has its full call-list known once every block
        # has been visited. Resolving now would see only the first step's PT.
        return stmt

    if t.startswith('__TON_RST__'):
        # Reset placeholders fold into the corresponding TON call's IN.
        return None

    return stmt


def _resolve_timer_placeholders(stmts: list[Statement], ctx: Context) -> list[Statement]:
    """Final pass: replace any surviving __TON__N__ placeholder statements with
    the merged TON FB call (PT-selected across steps; RST folded into IN)."""
    out = []
    for s in stmts:
        m = re.match(r'^__TON__(\d+)__$', s.text.strip())
        if m:
            out.append(_emit_ton_call(ctx, int(m.group(1))))
        else:
            out.append(s)
    return out


def _emit_ton_call(ctx: Context, tidx: int) -> Statement:
    """Emit a TON FB call.

    Handles a timer used in multiple STL steps with possibly-different PTs:
    the single FB instance gets IN = OR(all conds) and a PT chosen by whichever
    condition is currently active (via an IF/ELSIF prelude into a tn_pt var).
    Any RST Tn conditions are folded into IN as `... AND NOT (reset)`.
    """
    tname = safe_var_name(f"t{tidx}")
    calls = ctx.timer_calls.get(tidx, [])
    rsts = ctx.timer_reset_conds.get(tidx, [])

    def fold_reset(in_expr: str) -> str:
        if not rsts:
            return in_expr
        joined = " OR ".join(f"({r})" for r in rsts)
        return f"({in_expr}) AND NOT ({joined})"

    if not calls:
        # Shouldn't happen, but stay safe.
        return Statement('TIMER', f"{tname}(IN := FALSE, PT := T#0ms);",
                         scope='always')

    # Distinct PTs across all uses?
    distinct_pts = list(dict.fromkeys(pt for _c, pt in calls))

    if len(calls) == 1:
        cond, pt = calls[0]
        return Statement('TIMER',
                         f"{tname}(IN := {fold_reset(cond)}, PT := {pt});",
                         scope='always')

    # Multiple uses: OR the IN conditions.
    or_in = " OR ".join(f"({c})" for c, _pt in calls)

    if len(distinct_pts) == 1:
        # All PTs identical -> simple merge.
        pt = distinct_pts[0]
        return Statement('TIMER',
                         f"{tname}(IN := {fold_reset(or_in)}, PT := {pt});",
                         scope='always')

    # Different PTs -> select PT by whichever condition is active. Use a
    # dedicated tn_pt TIME variable, set via IF/ELSIF (STL steps are mutually
    # exclusive so at most one branch is taken per scan).
    pt_var = f"{tname}_pt"
    ctx.timer_pt_vars.add(pt_var)
    lines = []
    first = True
    for cond, pt in calls:
        kw = "IF" if first else "ELSIF"
        lines.append(f"{kw} {cond} THEN")
        lines.append(f"  {pt_var} := {pt};")
        first = False
    lines.append("END_IF;")
    lines.append(f"{tname}(IN := {fold_reset(or_in)}, PT := {pt_var});")
    return Statement('TIMER', "\n".join(lines), scope='always')


def _emit_ctu_call(ctx: Context, cidx: int) -> Statement:
    """Emit a CTU FB call, merging multiple OUT Cn uses (CU = OR of conds).
    PV uses the first declared value; a mismatch is surfaced as a lint note."""
    cname = safe_var_name(f"c{cidx}")
    calls = ctx.counter_calls.get(cidx, [])
    rsts = ctx.counter_reset_conds.get(cidx, [])
    if rsts:
        rstr = rsts[0] if len(rsts) == 1 else " OR ".join(f"({r})" for r in rsts)
    else:
        rstr = "FALSE"

    if not calls:
        pv = ctx.counter_pv.get(cidx, 0)
        return Statement('COUNTER',
                         f"{cname}(CU := FALSE, R := {rstr}, PV := {pv});",
                         scope='always')

    pvs = list(dict.fromkeys(pv for _c, pv in calls))
    pv = pvs[0]
    if len(calls) == 1:
        cu = calls[0][0]
    else:
        cu = " OR ".join(f"({c})" for c, _pv in calls)
    note = ""
    if len(pvs) > 1:
        note = f"  (* il2st: C{cidx} declared with differing PV {pvs}; using {pv} *)"
    return Statement('COUNTER',
                     f"{cname}(CU := {cu}, R := {rstr}, PV := {pv});{note}",
                     scope='always')


def _resolve_counter_placeholders(stmts: list[Statement], ctx: Context) -> list[Statement]:
    out = []
    for s in stmts:
        m = re.match(r'^__CTU__(\d+)__$', s.text.strip())
        if m:
            out.append(_emit_ctu_call(ctx, int(m.group(1))))
        else:
            out.append(s)
    return out


# ---------------------------------------------------------------------------
# 14. Variable declaration block assembly
# ---------------------------------------------------------------------------

def build_var_blocks(ctx: Context) -> str:
    """Build the ST VAR blocks. Located vs FB separated."""

    blocks: list[str] = []

    # ---- Located bools ----
    located_lines = []

    for i in sorted(ctx.x_used):
        # X uses Mitsubishi octal labelling (already normalised to decimal
        # bit-index 0..15 by parse_operand). Map to %QX0.x..%QX1.7.
        if i > 15:
            raise IL2STError(
                f"X bit-index {i} out of range. il2st maps X0..X17 (octal) "
                f"to %QX0.0..%QX1.7; higher addresses are not supported."
            )
        n = safe_var_name(_xy_canonical_name('X', i))
        addr = f"%QX{i // 8}.{i % 8}"  # bit 0..7 -> %QX0.x ; bit 8..15 -> %QX1.x
        located_lines.append(f"{n} AT {addr} : BOOL;")

    for i in sorted(ctx.y_used):
        # Y maps to %QX2.0..%QX3.7 (paired with X above; matches the user's
        # HMI loopback layout: sw0..sw15 on coils 0-15, lt0..lt15 on 16-31).
        if i > 15:
            raise IL2STError(
                f"Y bit-index {i} out of range. il2st maps Y0..Y17 (octal) "
                f"to %QX2.0..%QX3.7; higher addresses are not supported."
            )
        n = safe_var_name(_xy_canonical_name('Y', i))
        addr = f"%QX{2 + (i // 8)}.{i % 8}"
        located_lines.append(f"{n} AT {addr} : BOOL;")

    for i in sorted(ctx.m_used):
        n = safe_var_name(f"m{i}")
        byte = ADDR_M_BASE + i // 8
        if byte > ADDR_M_MAX_BYTE:
            raise IL2STError(
                f"M{i} -> byte {byte} exceeds the M zone "
                f"(%QX{ADDR_M_BASE}..%QX{ADDR_M_MAX_BYTE}). Too many M points; "
                f"the S zone begins at %QX{ADDR_S_BASE}.")
        located_lines.append(f"{n} AT %QX{byte}.{i % 8} : BOOL;")

    # S as BOOL (only used in parallel STL mode; in linear mode they
    # mirror "state = N"). S0..S1599 fit in the dedicated S zone.
    for i in sorted(ctx.s_used):
        n = safe_var_name(f"s{i}")
        byte = ADDR_S_BASE + i // 8
        if byte > ADDR_S_MAX_BYTE:
            raise IL2STError(
                f"S{i} -> byte {byte} exceeds the S zone "
                f"(%QX{ADDR_S_BASE}..%QX{ADDR_S_MAX_BYTE}).")
        located_lines.append(f"{n} AT %QX{byte}.{i % 8} : BOOL;")

    # special-relay decls (placed well above the S zone)
    if ctx.use_sm8002:
        located_lines.append(f"sm8002 AT %QX{ADDR_SM_BASE}.0 : BOOL;")
    if ctx.use_sm8013:
        located_lines.append(f"sm8013 AT %QX{ADDR_SM_BASE}.1 : BOOL;")

    if located_lines:
        blocks.append("VAR\n" + "\n".join(f"  {ln}" for ln in located_lines) + "\nEND_VAR")

    # ---- Located INTs (D registers) ----
    int_lines = []
    for i in sorted(ctx.d_used):
        n = safe_var_name(f"d{i}")
        addr = f"%QW{101 + i}"
        int_lines.append(f"{n} AT {addr} : INT;")
    # state variable (linear STL only) -- declare as INT located so monitor can see
    if getattr(ctx, '_linear_state_used', False):
        int_lines.append("state AT %QW100 : INT;")

    if int_lines:
        blocks.append("VAR\n" + "\n".join(f"  {ln}" for ln in int_lines) + "\nEND_VAR")

    # ---- TIME-typed D registers (used as timer PT) ----
    # Not located: TIME doesn't fit a single %QW cleanly and isn't supposed to
    # be Modbus-written directly. Initial value is T#0ms (timer is off until
    # a MOV K Dx assigns a real value).
    time_lines = []
    for i in sorted(ctx.d_pt_used):
        n = safe_var_name(f"d{i}")
        time_lines.append(f"{n} : TIME := T#0ms;")
    # PT-selector vars for timers shared across STL steps with different PTs.
    for pt_var in sorted(ctx.timer_pt_vars):
        time_lines.append(f"{pt_var} : TIME := T#0ms;")
    if time_lines:
        blocks.append("VAR\n" + "\n".join(f"  {ln}" for ln in time_lines) + "\nEND_VAR")

    # ---- FB instances (NOT located, must be separate VAR block) ----
    fb_lines = []
    for i in sorted(ctx.timer_used):
        n = safe_var_name(f"t{i}")
        fb_lines.append(f"{n} : TON;")
    for i in sorted(ctx.counter_used):
        n = safe_var_name(f"c{i}")
        fb_lines.append(f"{n} : CTU;")
    for trig_name in sorted(set(ctx.rtrig_for_pls.values())):
        fb_lines.append(f"{trig_name} : R_TRIG;")
    for trig_name in sorted(set(ctx.ftrig_for_plf.values())):
        fb_lines.append(f"{trig_name} : F_TRIG;")
    # Edge-detect trigs from LDP/LDF/ANDP/ANDF/ORP/ORF
    for (kind, _opd), trig_name in sorted(ctx.edge_trigs.items()):
        # Avoid double-declare if same name happens to be in rtrig_for_pls/ftrig_for_plf
        already = (trig_name in set(ctx.rtrig_for_pls.values()) or
                   trig_name in set(ctx.ftrig_for_plf.values()))
        if already:
            continue
        fb_type = 'R_TRIG' if kind == 'R' else 'F_TRIG'
        fb_lines.append(f"{trig_name} : {fb_type};")
    # SM8013 generator pair
    if ctx.use_sm8013:
        fb_lines.append("sm8013_t0 : TON;")
        fb_lines.append("sm8013_t1 : TON;")
    if fb_lines:
        blocks.append("VAR\n" + "\n".join(f"  {ln}" for ln in fb_lines) + "\nEND_VAR")

    # ---- SM8002 first-scan helper ----
    if ctx.use_sm8002:
        blocks.append("VAR\n  sm8002_first_scan : BOOL := TRUE;\nEND_VAR")

    # ---- Parallel-STL image-table shadow vars ----
    # `s_n_next` is the staging copy of `s_n` for the current scan. It must
    # be a plain (unlocated) BOOL: it's a scan-internal register, not
    # observable from the HMI / Modbus. See assemble()'s snapshot/commit.
    if ctx.parallel_stl and ctx.s_used:
        shadow_lines = []
        for sidx in sorted(ctx.s_used):
            sname = safe_var_name(f"s{sidx}")
            shadow_lines.append(f"{sname}_next : BOOL;")
        blocks.append("VAR\n" + "\n".join(f"  {ln}" for ln in shadow_lines) + "\nEND_VAR")

    return "\n".join(blocks)


def _bit_addr(area: str, byte: int, bit: int) -> str:
    """Compute a bit address that respects the 0..7 sub-bit rule."""
    return f"%{area}{byte + bit // 8}.{bit % 8}"


# ---------------------------------------------------------------------------
# 15. Special-relay helpers
# ---------------------------------------------------------------------------

def emit_special_relay_init(ctx: Context) -> list[Statement]:
    """Emit prologue statements for SM8002 and SM8013 synthesis."""
    out: list[Statement] = []
    if ctx.use_sm8002:
        # sm8002 := first_scan; first_scan := FALSE;
        out.append(Statement('ASSIGN', "sm8002 := sm8002_first_scan;"))
        out.append(Statement('ASSIGN', "sm8002_first_scan := FALSE;"))
    if ctx.use_sm8013:
        # 1 Hz square: 500 ms on / 500 ms off via two ping-pong TONs
        out.append(Statement('TIMER',
                             "sm8013_t0(IN := (NOT sm8013_t1.Q), PT := T#500ms);"))
        out.append(Statement('TIMER',
                             "sm8013_t1(IN := sm8013_t0.Q, PT := T#500ms);"))
        out.append(Statement('ASSIGN', "sm8013 := sm8013_t0.Q;"))
    return out


# ---------------------------------------------------------------------------
# 16. Assembly -- compose final .st text
# ---------------------------------------------------------------------------

PROGRAM_TEMPLATE = """\
PROGRAM {name}
{vars}

{body}
END_PROGRAM

CONFIGURATION Config0
  RESOURCE Res0 ON PLC
    TASK Main_Task(INTERVAL := T#{task_ms}ms, PRIORITY := 0);
    PROGRAM Inst0 WITH Main_Task : {name};
  END_RESOURCE
END_CONFIGURATION
"""


def assemble(ctx: Context,
             prelude: list[Statement],
             pre_stl: list[Statement],
             stl: list[Statement]) -> str:
    """Final assembly."""
    # Patch placeholders in pre_stl
    fixed_pre = []
    for s in pre_stl:
        ps = _postprocess_stmt(s, ctx,
                               state_map=getattr(ctx, '_linear_state_map', None))
        if ps is not None:
            fixed_pre.append(ps)

    # Final pass: resolve any remaining __TON__N__ placeholders now that the
    # complete timer_calls list is known across ALL STL blocks. (Doing this in
    # the per-block postprocess would only see the first step's PT.) Counters
    # are merged the same way (CU = OR of all uses).
    #
    # NOTE: this MUST run before build_var_blocks(), because resolving a timer
    # that is shared across steps with different PTs allocates a `tn_pt : TIME`
    # selector variable (ctx.timer_pt_vars) that the VAR block must declare.
    prelude = _resolve_timer_placeholders(prelude, ctx)
    fixed_pre = _resolve_timer_placeholders(fixed_pre, ctx)
    stl = _resolve_timer_placeholders(stl, ctx)
    prelude = _resolve_counter_placeholders(prelude, ctx)
    fixed_pre = _resolve_counter_placeholders(fixed_pre, ctx)
    stl = _resolve_counter_placeholders(stl, ctx)

    # Build VAR blocks AFTER placeholder resolution so tn_pt selectors declared
    # during resolution are included.
    var_blocks = build_var_blocks(ctx)

    body_lines = []

    # ---- Edge-trigger ticks: rtrig_x0(CLK := x0); etc. -----------------
    # Each R_TRIG / F_TRIG instance from LDP/LDF/ANDP/ANDF/ORP/ORF needs
    # to be "ticked" once per scan with the operand as CLK, so that .Q is
    # current when downstream expressions read it.
    if ctx.edge_trigs:
        for (kind, opd_str), trig_name in sorted(ctx.edge_trigs.items()):
            body_lines.append(f"  {trig_name}(CLK := {opd_str});")
        body_lines.append("")

    for s in prelude:
        for ln in s.text.splitlines():
            body_lines.append("  " + ln)
        body_lines.append("")

    # ---- Image-table snapshot (parallel STL only) ----------------------
    # Capture s_n at start of scan into s_n_next; pre-STL / IF s_n THEN blocks
    # write to the shadow only. A single commit at the bottom of the scan then
    # publishes s_n_next -> s_n. This enforces the Mitsubishi "one step per
    # scan" invariant: a SET S inside one IF block cannot make the next IF
    # block fire on the just-set state within the same scan.
    if ctx.parallel_stl and ctx.s_used:
        for sidx in sorted(ctx.s_used):
            sname = safe_var_name(f"s{sidx}")
            body_lines.append(f"  {sname}_next := {sname};")
        body_lines.append("")

    for s in fixed_pre:
        for ln in s.text.splitlines():
            body_lines.append("  " + ln)
        body_lines.append("")
    for s in stl:
        for ln in s.text.splitlines():
            body_lines.append("  " + ln)
        body_lines.append("")

    # ---- Image-table commit (parallel STL only) -----------------------
    if ctx.parallel_stl and ctx.s_used:
        for sidx in sorted(ctx.s_used):
            sname = safe_var_name(f"s{sidx}")
            body_lines.append(f"  {sname} := {sname}_next;")
        body_lines.append("")

    # collapse trailing blank lines
    while body_lines and not body_lines[-1].strip():
        body_lines.pop()

    body = "\n".join(body_lines)

    # Indent var_blocks to two-space prefix per line
    indented_vars = "\n".join("  " + ln for ln in var_blocks.splitlines())

    return PROGRAM_TEMPLATE.format(
        name=ctx.program_name,
        vars=indented_vars,
        body=body,
        task_ms=ctx.task_ms,
    )


# ---------------------------------------------------------------------------
# 17. Lint pass
# ---------------------------------------------------------------------------

# monitoring.py rules for located lines:
#   * line contains " AT "
#   * line contains "%"
#   * line does NOT contain "(*" or "*)"
#   * shape: NAME AT %ADDR : TYPE; (optional := default)
#     with EXACTLY one ASCII space between every token (no tabs, no double sp).
_LOCATED_RE = re.compile(
    r'^[A-Za-z_]\w* AT %[IQM][XWDLB]\d+(\.\d)? : [A-Z]+(\s*:=\s*\S+)?;\s*$'
)


def lint(st_text: str) -> list[str]:
    """
    Validate the generated ST for constitution compliance.
    Returns a list of warnings (strings); empty list means OK.
    """
    warnings = []
    for n, raw in enumerate(st_text.splitlines(), 1):
        line = raw.rstrip()
        stripped = line.strip()
        if ' AT ' in stripped and '%' in stripped \
                and '(*' not in stripped and '*)' not in stripped:
            # candidate located declaration -- must match strict regex
            if not _LOCATED_RE.match(stripped):
                warnings.append(
                    f"Line {n}: located variable does not match the "
                    f"single-space monitoring.py rule: {stripped!r}"
                )
        if '\t' in raw:
            warnings.append(f"Line {n}: contains a TAB character")

    # ---- Address-overlap check -----------------------------------------
    # Two located variables must never share the same %QX/%IX/%QW/... address.
    # This guards against zone-layout mistakes (e.g. S overrunning into the
    # SM zone, or M overlapping S).
    seen_addr: dict[str, str] = {}
    for n, raw in enumerate(st_text.splitlines(), 1):
        s = raw.strip()
        m = re.match(r'^([A-Za-z_]\w*)\s+AT\s+(%\w+[\d.]*)\s*:', s)
        if m:
            var, addr = m.group(1), m.group(2)
            if addr in seen_addr and seen_addr[addr] != var:
                warnings.append(
                    f"Line {n}: address {addr} already used by "
                    f"{seen_addr[addr]!r}; {var!r} collides with it")
            else:
                seen_addr[addr] = var

    # ---- Undeclared-variable check -------------------------------------
    # Collect every identifier declared in any VAR...END_VAR block, then make
    # sure each assignment LHS and each FB-call target was actually declared.
    # This catches code-generation ordering bugs (e.g. a tn_pt selector used
    # but never declared) before they reach the MATIEC compiler.
    declared: set[str] = set()
    in_var = False
    in_config = False
    for raw in st_text.splitlines():
        s = raw.strip()
        if re.match(r'^CONFIGURATION\b', s):
            in_config = True
        if in_config:
            continue
        if re.match(r'^VAR(\s|$)', s):
            in_var = True
            continue
        if s == 'END_VAR':
            in_var = False
            continue
        if in_var:
            m = re.match(r'^([A-Za-z_]\w*)', s)
            if m:
                declared.add(m.group(1).lower())

    _ST_KEYWORDS = {
        'if', 'then', 'elsif', 'else', 'end_if', 'case', 'of', 'end_case',
        'for', 'to', 'by', 'do', 'end_for', 'while', 'end_while', 'repeat',
        'until', 'end_repeat', 'return', 'exit', 'and', 'or', 'xor', 'not',
        'mod', 'true', 'false',
    }
    in_config = False
    for n, raw in enumerate(st_text.splitlines(), 1):
        s = raw.strip()
        if re.match(r'^(PROGRAM|CONFIGURATION)\b', s):
            if s.startswith('CONFIGURATION'):
                in_config = True
        if in_config or not s or s.startswith('(*'):
            continue
        # Skip declaration lines inside VAR blocks (they contain ':=' too).
        if ' AT ' in s or re.match(r'^[A-Za-z_]\w*\s*:\s*(BOOL|INT|TIME|TON|CTU|R_TRIG|F_TRIG|DINT|REAL|WORD)\b', s):
            continue
        # Assignment LHS: `name :=`
        m = re.match(r'^([A-Za-z_]\w*)\s*:=', s)
        if m:
            name = m.group(1).lower()
            if name not in declared and name not in _ST_KEYWORDS:
                warnings.append(
                    f"Line {n}: assignment to undeclared variable {name!r}")
        # FB call: `name(` with no space before paren (named-param invocation).
        m = re.match(r'^([A-Za-z_]\w*)\(', s)
        if m:
            name = m.group(1).lower()
            if name not in declared and name not in _ST_KEYWORDS:
                warnings.append(
                    f"Line {n}: call to undeclared function block {name!r}")

    # ---- Orphan FB check ------------------------------------------------
    # A function block (TON/CTU/R_TRIG/F_TRIG) declared but never called means
    # its placeholder was dropped during code-gen -- its .Q/.CV will read as a
    # constant 0/FALSE, silently breaking dependent logic (e.g. an oscillator).
    fb_decls: dict[str, int] = {}
    fb_called: set[str] = set()
    in_config2 = False
    for n, raw in enumerate(st_text.splitlines(), 1):
        s = raw.strip()
        if re.match(r'^CONFIGURATION\b', s):
            in_config2 = True
        if in_config2:
            continue
        m = re.match(r'^([A-Za-z_]\w*)\s*:\s*(TON|CTU|R_TRIG|F_TRIG)\b', s)
        if m:
            fb_decls[m.group(1).lower()] = n
            continue
        for cm in re.finditer(r'\b([A-Za-z_]\w*)\(', s):
            fb_called.add(cm.group(1).lower())
    for name, ln in sorted(fb_decls.items(), key=lambda kv: kv[1]):
        if name not in fb_called:
            warnings.append(
                f"Line {ln}: function block {name!r} declared but never called "
                f"-- output stuck at 0/FALSE (a dropped timer/counter call?).")

    return warnings


# ---------------------------------------------------------------------------
# 18. Top-level convert()
# ---------------------------------------------------------------------------

def _check_startup_interlock(pre_stl: list, blocks: list) -> list[str]:
    """Detect a startup-interlock M that is never cleared on a normal end path.

    Pattern: a program guards its start with `ANI Mn` before the first `SET S`
    (so Mn=ON blocks re-start), and `SET Mn` somewhere locks it. If Mn is only
    ever RST in the pre-STL section (i.e. an X0-style global stop) and never
    inside any STL state, then finishing the sequence normally (e.g. via a stop
    button leading to a terminal state) will leave Mn latched -- the machine
    cannot be restarted with the start button until the emergency stop is hit.

    This is a *lint hint*, not an auto-fix: il2st never silently rewrites your
    control logic. It just flags the likely trap.
    """
    warnings: list[str] = []

    # 1. startup-interlock M = an `ANI Mn` appearing before the first SET S.
    interlock_m: set[int] = set()
    seen_first_set_s = False
    for instr in pre_stl:
        if instr.op == 'SET' and instr.args:
            try:
                pfx, _idx, _ = parse_operand(instr.args[0])
                if pfx == 'S':
                    seen_first_set_s = True
            except Exception:
                pass
        if not seen_first_set_s and instr.op == 'ANI' and instr.args:
            try:
                pfx, idx, _ = parse_operand(instr.args[0])
                if pfx == 'M':
                    interlock_m.add(idx)
            except Exception:
                pass
    if not interlock_m:
        return warnings

    # 2. which of those M are RST inside ANY STL block?
    rst_in_stl: set[int] = set()
    for b in blocks:
        for instr in b.instrs:
            if instr.op == 'RST' and instr.args:
                try:
                    pfx, idx, _ = parse_operand(instr.args[0])
                    if pfx == 'M':
                        rst_in_stl.add(idx)
                except Exception:
                    pass

    # 3. interlock M never RST in any state -> normal end won't release it.
    for m in sorted(interlock_m):
        if m not in rst_in_stl:
            warnings.append(
                f"M{m} 用作啟動互鎖(啟動前有 ANI M{m}),但沒有任何 STL 狀態用 "
                f"RST M{m} 清除它 -- 流程正常結束(例如停止鈕進入結束狀態)後 M{m} "
                f"仍鎖定,將無法用啟動鈕重啟,只能靠緊急停止清除。若要結束後能重啟,"
                f"在結束狀態的 IL 加一行 RST M{m}。")
    return warnings


def convert(csv_text: str,
            program_name: str = 'Prac',
            task_ms: int = 20) -> str:
    """Main entry point: csv text -> .st text."""
    instrs = parse_csv(csv_text)
    validate_instructions(instrs)

    ctx = Context(program_name=program_name, task_ms=task_ms)

    pre_stl, stl_blocks = pre_scan_stl(instrs, ctx)

    pre_stl_stmts = translate_pre_stl(pre_stl, ctx)
    stl_stmts = translate_stl(stl_blocks, ctx)

    # Inject special-relay synthesis as the very first statements.
    prelude = emit_special_relay_init(ctx)

    st_text = assemble(ctx, prelude, pre_stl_stmts, stl_stmts)

    # lint and raise if anything failed strictly
    issues = lint(st_text)
    # logic-level hint: startup interlock never cleared on a normal end path
    issues = issues + _check_startup_interlock(pre_stl, stl_blocks)
    if issues:
        # Append warnings as block comment AFTER the configuration
        warn_block = "\n(* il2st LINT WARNINGS:\n" + \
                     "\n".join(f"   - {w}" for w in issues) + "\n*)\n"
        st_text = st_text + warn_block

    return st_text


# ---------------------------------------------------------------------------
# 19. CLI
# ---------------------------------------------------------------------------

def main(argv=None):
    p = argparse.ArgumentParser(
        description="Convert FX5U Instruction List CSV to OpenPLC v3 ST",
        epilog="Output: .st file ready for upload via OpenPLC webserver."
    )
    p.add_argument('input', help='Input CSV file')
    p.add_argument('-o', '--output', help='Output .st file (default: stdout)')
    p.add_argument('-n', '--name', default='Prac',
                   help='ST program name (default: Prac)')
    p.add_argument('--task-ms', type=int, default=20,
                   help='Task interval in milliseconds (default: 20)')
    args = p.parse_args(argv)

    # Auto-detect UTF-16 (GX Works3 export); fall back to UTF-8.
    csv_text = None
    for enc in ('utf-8-sig', 'utf-16', 'utf-16-le', 'utf-16-be'):
        try:
            with open(args.input, 'r', encoding=enc) as f:
                csv_text = f.read()
            break
        except (UnicodeDecodeError, UnicodeError):
            continue
    if csv_text is None:
        print(f"il2st: ERROR: could not decode {args.input}", file=sys.stderr)
        sys.exit(2)

    try:
        st = convert(csv_text, program_name=args.name, task_ms=args.task_ms)
    except IL2STError as e:
        print(f"il2st: ERROR: {e}", file=sys.stderr)
        sys.exit(2)

    if args.output:
        with open(args.output, 'w', encoding='utf-8', newline='\n') as f:
            f.write(st)
    else:
        sys.stdout.write(st)


if __name__ == '__main__':
    main()

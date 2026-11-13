# Copyright 2026 EdgeAI Robotics Challenge Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Rule baseline: turn a natural-language instruction into candidate mission steps.

No model is involved. The output is mission semantics only (targets, order,
dependencies, priority); robots, poses, paths and speeds never appear in it.
The interpreter does not validate: unknown targets and cycles are passed on
faithfully so the validator can reject them with its own codes.

How an instruction is read:

1. Whole-instruction checks give up early: empty text or no mission content
   (12), injection attempts and low-level motion such as speeds or
   coordinates (12), conditions or fallbacks (13), vague references such as
   "the closest target" (13).
2. The text is split into sentences and comma clauses. Each clause is one of:
   a condition ("After T2 is done", "T2 完成后"), a relation ("T1 after T3",
   "T1 在 T2 之后救援"), a priority statement, an exclusion ("不要救援 T2"),
   or a group of targets to rescue, optionally led by an order marker
   ("first", "then", "再", "最后").
3. A group led by "then"-like markers follows the previous group; a group
   after a condition follows the condition's targets; clauses in the same
   sentence without a marker join the current group.
4. The four unsettled cases (or-dependency, time limit, named robot, same
   target twice) are resolved by the policy table, see ``policy.py``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import re
from typing import Any, Mapping, Sequence
import unicodedata

from .policy import load_policy, Policy

# Values copied from src/edgeai_robot_interfaces/msg/MissionValidationIssue.msg;
# test_issue_codes.py compares them with the file.
CODE_OUTPUT_UNSTRUCTURED = 12
CODE_INSTRUCTION_AMBIGUOUS = 13

# schemas/mission_plan.schema.json, $defs/step.
ACTION_RESCUE = 'rescue'
STEP_FIELDS = ('step_id', 'action', 'target_id', 'priority', 'predecessor_step_ids')

# target_type values of schemas/target_list.schema.json.
NORMAL = 'normal'
COOPERATIVE = 'cooperative'

INTERPRETER_NAME = 'rule_baseline'
INTERPRETER_VERSION = '0.1.0'


@dataclass(frozen=True)
class Issue:
    """Mirrors the fields of MissionValidationIssue.msg."""

    code: int
    step_id: str = ''
    target_id: str = ''
    detail: str = ''


@dataclass
class InterpretationResult:
    ok: bool
    steps: list[dict] = field(default_factory=list)
    issues: list[Issue] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            'ok': self.ok,
            'steps': [dict(s) for s in self.steps],
            'issues': [vars(i).copy() for i in self.issues],
            'notes': list(self.notes),
        }


class _GiveUp(Exception):

    def __init__(self, code: int, detail: str):
        super().__init__(detail)
        self.code = code
        self.detail = detail


def registry_from_target_list(document: Mapping[str, Any]) -> dict[str, str]:
    """Build {target_id: target_type} from a frozen target list (target_list.schema.json)."""
    return {t['target_id']: t['target_type'] for t in document['targets']}


# --------------------------------------------------------------------- patterns

_TARGET = re.compile(r'(?<![A-Za-z0-9_])[Tt]([Cc]?)(\d+)(?![A-Za-z0-9_])')
_SENTENCE_SPLIT = re.compile(r'[。;!?\n]|\.(?!\d)')

_INJECTION = re.compile(
    r'ignore\s+(?:all\s+)?(?:the\s+)?(?:previous|prior|above|earlier)|disregard\s+'
    r'|system\s+prompt|忽略(?:之前|以上|前面|上面|所有)|系统提示', re.I)
_MOTION = re.compile(
    r'\d+(?:\.\d+)?\s*(?:m/s|米每秒|米/秒|meters?\s+per\s+second)|速度|\bspeed\b|\bvelocity\b'
    r'|坐标|\bcoordinates?\b|\(\s*-?\d+(?:\.\d+)?\s*,\s*-?\d+(?:\.\d+)?\s*\)'
    r'|\bforward\b|\bbackward\b|\bturn\s+(?:left|right)\b|\bdrive\b|左转|右转|前进|后退|开到'
    r'|\d+(?:\.\d+)?\s*(?:米|meters?|metres?)(?![A-Za-z])|路径|\bpath\b|\broute\b|\bwaypoints?\b',
    re.I)
_CONDITIONAL = re.compile(
    r'\bif\b|\bunless\b|\botherwise\b|\binstead\b|\bfail(?:s|ed|ure)?\b'
    r'|如果|假如|要是|万一|除非|否则|失败|改救|改为救', re.I)
_VAGUE = re.compile(
    r'那个目标|这个目标|该目标|某个目标|一个(?:普通|协同)?目标|另一个|另外一个|最近|最远'
    r'|\b(?:closest|nearest|farthest)\b'
    r'|\b(?:a|another|some|that|this|the\s+other|the)\s+target\b', re.I)
_MISSION_WORDS = re.compile(r'救|目标|\brescue|\bsave\b|\btargets?\b', re.I)

_NAMED_ROBOT = re.compile(
    r'\brobot[_\s]?\d+\b|机器人\s*[_#]?\s*\d+|\d+\s*号\s*机器人', re.I)
_TIME_LIMIT = re.compile(
    r'(?:在\s*)?\d+(?:\.\d+)?\s*(?:分钟|分|秒钟|秒|小时)\s*(?:之内|以内|内)'
    r'|\bwithin\s+\d+(?:\.\d+)?\s*(?:minutes?|mins?|seconds?|secs?|hours?)\b'
    r'|\bin\s+(?:under|less\s+than)\s+\d+(?:\.\d+)?\s*(?:minutes?|mins?|seconds?|secs?|hours?)\b'
    r'|\btime\s+limit\b|限时', re.I)
_OR = re.compile(r'或者|或|\bor\b|\beither\b|任意一个|任一|\bany\s+one\s+of\b|\bwhichever\b', re.I)
_TWICE = re.compile(r'\btwice\b|两次|二次|2\s*次', re.I)

_EXCLUDE = re.compile(
    r'除了|除|\bexcept(?:\s+for)?\b|\bother\s+than\b|\bbut\s+not\b|\bexcluding\b'
    r'|不要|不用|不必|无需|\bdo\s+not\b|\bdon\'t\b|\bnever\b|\bskip\b', re.I)

_SELECT_NORMAL = re.compile(
    r'(?:所有|全部|其他|其余|其它)的?普通目标|\ball\s+(?:the\s+)?(?:other\s+)?normal\s+targets\b'
    r'|\bevery\s+normal\s+target\b', re.I)
_SELECT_COOPERATIVE = re.compile(
    r'(?:所有|全部|其他|其余|其它)的?协同目标'
    r'|\ball\s+(?:the\s+)?(?:other\s+)?cooperative\s+targets\b'
    r'|\bevery\s+cooperative\s+target\b', re.I)
_SELECT_ALL = re.compile(
    r'(?:所有|全部|其他|其余|其它)的?目标|\ball\s+(?:the\s+)?(?:other\s+|remaining\s+)?targets\b'
    r'|\bevery\s+target\b|\bthe\s+rest\b', re.I)

_LEAD_FIRST = re.compile(r'^(?:请)?\s*(?:先|首先)|^(?:first(?:ly)?|first\s+of\s+all)\b', re.I)
_LEAD_THEN = re.compile(
    r'^(?:请)?\s*(?:然后|再|接着|随后|最后|之后|而后|接下来)'
    r'|^(?:and\s+)?(?:then|finally|afterwards|after\s+that|next|lastly)\b', re.I)
_MID_THEN = re.compile(r'\s*(?=然后|接着|随后|最后)|\s+(?=(?:and\s+)?then\b)', re.I)

_CHAIN = re.compile(
    r'依次|顺序(?:是|为)|按(?:照)?(?:以下|如下|这个|下面的)?顺序'
    r'|\bone\s+(?:by|after)\s+(?:one|another)\b|\bin\s+(?:this|that|the\s+following)\s+order\b'
    r'|\bin\s+sequence\b|\bsequentially\b', re.I)

_PRIORITY_NONE = re.compile(
    r'没有优先级|无优先级|不分优先|优先级相同|同等优先|优先级一样'
    r'|\bno\s+priority\b|\bsame\s+priority\b|\bequal\s+priority\b|\bwithout\s+priority\b', re.I)
_PRIORITY = re.compile(
    r'优先级|最重要|最紧急|最不重要|最优先|优先救援'
    r'|\bpriority\b|\bimportant\b|\burgent\b', re.I)
_PRIORITY_TOP = re.compile(
    r'最高|最重要|最紧急|最优先|优先救援'
    r'|\bhighest\b|\bmost\s+(?:important|urgent)\b|\btop\s+priority\b|\bfirst\s+priority\b', re.I)
_PRIORITY_BOTTOM = re.compile(
    r'最低|最不重要|最不紧急|\blowest\b|\bleast\s+(?:important|urgent)\b', re.I)

_RESCUE_VERB = re.compile(r'救|\brescue\b|\bsave\b', re.I)

# Conditions and relations.
_EN_COND = re.compile(r'^(?:and\s+)?(?:only\s+)?(?:after|once|when)\b(?P<rest>.*)$', re.I)
_EN_BEFORE_LEAD = re.compile(r'^(?:and\s+)?before\b(?P<rest>.*)$', re.I)
_EN_DONE = re.compile(
    r'\b(?:is|are|has\s+been|have\s+been)\s+(?:all\s+)?(?:completed|done|finished|rescued)\b'
    r'|\b(?:completed|done|finished|rescued)\b', re.I)
_EN_ACTION_START = re.compile(r'\b(?:rescue|save|use|go)\b', re.I)
_EN_AFTER_MID = re.compile(r'\bafter\b', re.I)
_EN_BEFORE_MID = re.compile(r'\bbefore\b', re.I)
_ZH_AFTER = re.compile(
    r'(?:都|全部|均)?\s*(?:被)?(?:救援)?(?:完成|结束|救完|做完)?\s*(?:了)?\s*'
    r'(?:之后|以后|(?<![最然随而先])后)')
_ZH_BEFORE = re.compile(r'(?:之前|以前)')
_REFERENCE = re.compile(
    r'两者|二者|三者|它们|这些|以上|上述|前面|都|全部'
    r'|\ball\s+(?:three|four|five|of\s+them|these|those)\b|\bboth\b|\bthem\b|\bthese\b'
    r'|\bthose\b|\bthey\b|\ball\b', re.I)


def normalize(text: str) -> str:
    """NFKC (full-width to ASCII), upper-case target ids, collapse whitespace."""
    text = unicodedata.normalize('NFKC', text or '')
    text = _TARGET.sub(lambda m: 'T' + m.group(1).upper() + m.group(2), text)
    return re.sub(r'\s+', ' ', text).strip()


def _targets_in(text: str) -> list[str]:
    out: list[str] = []
    for m in _TARGET.finditer(text):
        tid = 'T' + m.group(1).upper() + m.group(2)
        if tid not in out:
            out.append(tid)
    return out


# ------------------------------------------------------------------------ parser

class _Parser:

    def __init__(self, registry: Mapping[str, str]):
        self.registry = dict(registry)
        self.node_target: dict[str, str] = {}       # node key -> target id, in creation order
        self.edges: list[tuple[str, str]] = []      # (before, after) node keys
        self.latest: dict[str, str] = {}            # target -> latest node key
        self.action_group: dict[str, int] = {}      # target -> group id of latest action
        self.action_targets: set[str] = set()
        self.duplicates: list[str] = []             # node keys of repeated actions
        self.excluded: set[str] = set()
        self.or_operands: list[list[str]] = []
        self.time_limited: list[tuple[list[str], str]] = []
        self.priority_marks: list[tuple[str, str]] = []   # (target, 'top'|'next'|'bottom')
        self.group_id = 0
        self.prev_group: list[str] = []

    # nodes -------------------------------------------------------------------

    def _ref(self, target: str) -> str:
        if target not in self.latest:
            self.latest[target] = target
            self.node_target[target] = target
        return self.latest[target]

    def _action(self, target: str, group: int) -> str:
        if target in self.action_group and self.action_group[target] != group:
            n = sum(1 for t in self.node_target.values() if t == target) + 1
            key = f'{target}#{n}'
            self.node_target[key] = target
            self.latest[target] = key
            self.duplicates.append(key)
        else:
            key = self._ref(target)
        self.action_group[target] = group
        self.action_targets.add(target)
        return key

    def _edge(self, before: str, after: str):
        if (before, after) not in self.edges:
            self.edges.append((before, after))

    def _new_group(self) -> int:
        self.group_id += 1
        return self.group_id

    def _add_group(self, targets: Sequence[str], preds: Sequence[str],
                   group: int | None = None) -> list[str]:
        group = self._new_group() if group is None else group
        keys = []
        for t in targets:
            key = self._action(t, group)
            for p in preds:
                self._edge(p, key)
            keys.append(key)
        return keys

    # selection ---------------------------------------------------------------

    def _select(self, text: str) -> list[str]:
        """Explicit target ids plus registry selections, in order of appearance."""
        found: list[tuple[int, str]] = []
        for m in _TARGET.finditer(text):
            found.append((m.start(), 'T' + m.group(1).upper() + m.group(2)))
        taken = []
        for pattern, kind in ((_SELECT_NORMAL, NORMAL), (_SELECT_COOPERATIVE, COOPERATIVE),
                              (_SELECT_ALL, None)):
            for m in pattern.finditer(text):
                if any(a <= m.start() < b for a, b in taken):
                    continue
                taken.append((m.start(), m.end()))
                if not self.registry:
                    raise _GiveUp(CODE_INSTRUCTION_AMBIGUOUS,
                                  '指令按类别选目标，但没有目标清单')
                for tid, ttype in self.registry.items():
                    if kind is None or ttype == kind:
                        found.append((m.start(), tid))
        out: list[str] = []
        for _, tid in sorted(found, key=lambda x: x[0]):
            if tid not in out:
                out.append(tid)
        return out

    def _reference_or_targets(self, text: str) -> list[str]:
        targets = self._select(text)
        if targets:
            return [self._ref(t) for t in targets]
        if _REFERENCE.search(text) and self.prev_group:
            return list(self.prev_group)
        raise _GiveUp(CODE_INSTRUCTION_AMBIGUOUS, '前置条件没有指明是哪些目标')

    def _condition(self, text: str) -> list[str]:
        if _OR.search(text):
            operands = self._select(text)
            if len(operands) < 2:
                raise _GiveUp(CODE_INSTRUCTION_AMBIGUOUS, '「或」前置没有指明目标')
            self.or_operands.append(operands)
        return self._reference_or_targets(text)

    # clauses -----------------------------------------------------------------

    def parse(self, text: str):
        for sentence in _SENTENCE_SPLIT.split(text):
            sentence = sentence.strip(' ,')
            if sentence:
                self._sentence(sentence)

    def _sentence(self, sentence: str):
        clauses = []
        for part in sentence.split(','):
            clauses += [c.strip() for c in _MID_THEN.split(part) if c.strip()]
        if _CHAIN.search(sentence) and not _OR.search(sentence):
            self._chain(clauses)
            return
        current: list[str] = []          # keys of the group this sentence is building
        current_id: int | None = None
        current_preds: list[str] = []
        pending_preds: list[str] | None = None
        pending_succ: list[str] = []
        pending_link = False
        in_priority = False
        for clause in clauses:
            clause = self._strip_time_limit(clause)
            body = clause
            m = _EXCLUDE.search(body)
            if m:
                self.excluded.update(self._select(body[m.end():]))
                body = body[:m.start()].strip()
                if not body:
                    continue
            if _PRIORITY_NONE.search(body):
                continue
            if _PRIORITY.search(body) or (in_priority and not _RESCUE_VERB.search(body)
                                          and self._select(body)):
                in_priority = self._priority(body, in_priority)
                continue
            lead_then = bool(_LEAD_THEN.match(body))
            lead_first = bool(_LEAD_FIRST.match(body))
            relation = self._relation(body)
            if relation is not None:
                kind, left, right = relation
                if kind == 'cond':
                    preds = self._condition(left)
                    targets = self._select(right)
                    if targets:
                        current_id = self._new_group()
                        current = self._add_group(targets, preds, current_id)
                        current_preds = preds
                        self.prev_group = current
                        pending_preds = None
                    else:
                        pending_preds = preds
                    continue
                if kind == 'before_lead':
                    succ = [self._ref(t) for t in self._select(left)]
                    targets = self._select(right)
                    if targets:
                        keys = self._add_group(targets, [])
                        for k in keys:
                            for s in succ:
                                self._edge(k, s)
                        self.prev_group = keys
                    else:
                        pending_succ = succ
                    continue
                if kind == 'after':      # left happens after right
                    preds = self._reference_or_targets(right)
                    keys = self._add_group(self._select(left), preds)
                    self.prev_group = keys
                    continue
                if kind == 'before':     # left happens before right
                    keys = self._add_group(self._select(left), [])
                    for t in self._select(right):
                        s = self._ref(t)
                        for k in keys:
                            self._edge(k, s)
                    self.prev_group = keys
                    continue
            targets = self._select(body)
            if not targets:
                if lead_then:
                    pending_link = True
                continue
            if _OR.search(body):
                raise _GiveUp(CODE_INSTRUCTION_AMBIGUOUS, '「或」选目标，没说清救哪个')
            if pending_preds is not None:
                preds, pending_preds = pending_preds, None
            elif lead_then or pending_link:
                preds = list(self.prev_group)
            elif current_id is not None and not lead_first:
                keys = self._add_group(targets, current_preds, current_id)
                current += [k for k in keys if k not in current]
                self.prev_group = current
                continue
            else:
                preds = []
            pending_link = False
            current_id = self._new_group()
            current = self._add_group(targets, preds, current_id)
            current_preds = preds
            for k in current:
                for s in pending_succ:
                    self._edge(k, s)
            pending_succ = []
            if _TWICE.search(body):
                current_id = self._new_group()
                current = self._add_group(targets, current, current_id)
            self.prev_group = current

    def _chain(self, clauses: Sequence[str]):
        link = bool(clauses) and bool(_LEAD_THEN.match(clauses[0]))
        prev = list(self.prev_group) if link else []
        for t in self._select(' '.join(clauses)):
            prev = self._add_group([t], prev)
        self.prev_group = prev

    def _relation(self, body: str) -> tuple[str, str, str] | None:
        m = _EN_COND.match(body)
        if m:
            rest = m.group('rest')
            done = _EN_DONE.search(rest)
            if done:
                return 'cond', rest[:done.start()], rest[done.end():]
            act = _EN_ACTION_START.search(rest)
            if act:
                return 'cond', rest[:act.start()], rest[act.start():]
            return 'cond', rest, ''
        m = _EN_BEFORE_LEAD.match(body)
        if m:
            rest = m.group('rest')
            parts = re.split(r'\b(?:you|we)\s+(?:rescue|save)\b', rest, maxsplit=1, flags=re.I)
            rest = parts[-1]
            act = _EN_ACTION_START.search(rest)
            if act and self._select(rest[:act.start()]):
                return 'before_lead', rest[:act.start()], rest[act.start():]
            return 'before_lead', rest, ''
        m = _EN_AFTER_MID.search(body)
        if m and self._select(body[:m.start()]):
            return 'after', body[:m.start()], body[m.end():]
        m = _EN_BEFORE_MID.search(body)
        if m and self._select(body[:m.start()]):
            return 'before', body[:m.start()], body[m.end():]
        m = _ZH_BEFORE.search(body)
        if m:
            left, right = body[:m.start()], body[m.end():]
            if '在' in left:
                subject, _, other = left.rpartition('在')
                if self._select(subject):
                    return 'before', subject, other
                if self._select(other):
                    return 'before_lead', other, right
        m = _ZH_AFTER.search(body)
        if m:
            left, right = body[:m.start()], body[m.end():]
            if '在' in left:
                subject, _, other = left.rpartition('在')
                if self._select(subject) and not self._select(right):
                    return 'after', subject, other
                left = other
            if self._select(left) or (_REFERENCE.search(left) and self.prev_group):
                return 'cond', left, right
        return None

    def _priority(self, body: str, continuing: bool) -> bool:
        targets = self._select(body)
        if _PRIORITY_BOTTOM.search(body):
            mark = 'bottom'
        elif _PRIORITY_TOP.search(body):
            mark = 'top'
        elif continuing:
            mark = 'next'
        else:
            mark = 'top'
        for t in targets:
            self._ref(t)
            self.priority_marks.append((t, mark))
        return True

    def _strip_time_limit(self, clause: str) -> str:
        m = _TIME_LIMIT.search(clause)
        if not m:
            return clause
        targets = self._select(clause)
        self.time_limited.append((targets, m.group(0).strip()))
        return (clause[:m.start()] + ' ' + clause[m.end():]).strip()


# ------------------------------------------------------------------- public API

def _fail(code: int, detail: str, notes: Sequence[str] = ()) -> InterpretationResult:
    return InterpretationResult(ok=False, issues=[Issue(code=code, detail=detail)],
                                notes=list(notes))


def _priorities(marks: Sequence[tuple[str, str]]) -> dict[str, int]:
    rank: dict[str, int] = {}
    last = -1
    for target, mark in marks:
        if mark == 'top':
            rank[target] = 0
            last = 0
        elif mark == 'next':
            last += 1
            rank[target] = last
    bottom = max(rank.values(), default=-1) + 1
    for target, mark in marks:
        if mark == 'bottom':
            rank[target] = bottom
    if not rank:
        return {}
    top = max(rank.values()) + 1
    return {t: top - r for t, r in rank.items()}


def interpret(instruction: str, registry: Mapping[str, str] | None = None,
              policy: Policy | None = None) -> InterpretationResult:
    """
    Interpret one instruction.

    ``registry`` maps target_id to target_type (normal or cooperative) of the
    frozen target list; it is only used to expand selections such as "all
    normal targets". ``policy`` defaults to config/policy.yaml.
    """
    policy = policy or load_policy()
    registry = registry or {}
    text = normalize(instruction)
    notes: list[str] = []

    if not _TARGET.search(text) and not _MISSION_WORDS.search(text):
        return _fail(CODE_OUTPUT_UNSTRUCTURED, '指令里没有任务内容')
    if _INJECTION.search(text):
        return _fail(CODE_OUTPUT_UNSTRUCTURED, '指令试图改变解释器的规则')
    if _MOTION.search(text):
        return _fail(CODE_OUTPUT_UNSTRUCTURED, '指令含底层运动内容，解释器只产出任务语义')
    if _CONDITIONAL.search(text):
        return _fail(CODE_INSTRUCTION_AMBIGUOUS, '指令含条件或备选，计划格式表达不了')
    if _VAGUE.search(text):
        return _fail(CODE_INSTRUCTION_AMBIGUOUS, '指令没有明确指出目标')

    if _NAMED_ROBOT.search(text):
        if policy.named_robot == 'reject':
            return _fail(CODE_INSTRUCTION_AMBIGUOUS, '指令点名了机器人（策略 named_robot=reject）')
        text = _NAMED_ROBOT.sub(' ', text)
        notes.append('已去掉指令里点名的机器人，选机器人归协调器（策略 named_robot=drop_robot）')

    parser = _Parser(registry)
    try:
        parser.parse(text)
    except _GiveUp as err:
        return _fail(err.code, err.detail, notes)

    if not parser.node_target:
        code = CODE_INSTRUCTION_AMBIGUOUS if _MISSION_WORDS.search(text) \
            else CODE_OUTPUT_UNSTRUCTURED
        return _fail(code, '没有找到要救援的目标', notes)

    if parser.or_operands:
        if policy.or_dependency == 'reject':
            return _fail(CODE_INSTRUCTION_AMBIGUOUS,
                         '「或」依赖，前置只能表达「全部完成」（策略 or_dependency=reject）', notes)
        for operands in parser.or_operands:
            if not set(operands) <= parser.action_targets:
                return _fail(CODE_INSTRUCTION_AMBIGUOUS,
                             '「或」依赖的目标不都在计划里，不能收紧成「都完成」', notes)
        notes.append('「或」依赖已收紧成「都完成」（策略 or_dependency=tighten_to_all）')

    if parser.time_limited and policy.time_limit == 'reject':
        return _fail(CODE_INSTRUCTION_AMBIGUOUS, '指令有时间限制（策略 time_limit=reject）', notes)

    node_target = dict(parser.node_target)
    edges = list(parser.edges)

    for key in parser.duplicates:
        target = node_target[key]
        first = target
        if policy.duplicate_target == 'merge':
            edges = [(a, b) for a, b in edges if b != key]
            edges = [(first if a == key else a, b) for a, b in edges]
            del node_target[key]
            notes.append(f'{target} 出现两次，已合并成一步（策略 duplicate_target=merge）')
        else:
            edges.append((first, key))
            notes.append(f'{target} 出现两次，保留两步，第二步在第一步之后'
                         '（策略 duplicate_target=keep_both）')

    for key in [k for k, t in node_target.items() if t in parser.excluded]:
        del node_target[key]
    edges = [(a, b) for a, b in edges if a in node_target and b in node_target]
    if not node_target:
        return _fail(CODE_INSTRUCTION_AMBIGUOUS, '排除之后没有剩下要救援的目标', notes)

    priority = dict.fromkeys(node_target, 0)
    by_target = _priorities(parser.priority_marks)
    for key, target in node_target.items():
        priority[key] = by_target.get(target, 0)
    for targets, phrase in parser.time_limited:
        keys = [k for k, t in node_target.items() if t in targets] or list(node_target)
        top = max(priority.values()) + 1
        for k in keys:
            priority[k] = top
        notes.append(f'时间限制「{phrase}」计划表达不了，原文照录；'
                     '这一步的优先级已调到最高（策略 time_limit=raise_priority）')

    step_id = {}
    for key, target in node_target.items():
        suffix = '-' + key.split('#')[1] if '#' in key else ''
        step_id[key] = f'step-{target.lower()}{suffix}'
    order = list(node_target)
    steps = []
    for key in order:
        preds = [step_id[a] for a in order if (a, key) in edges]
        steps.append({
            'step_id': step_id[key],
            'action': ACTION_RESCUE,
            'target_id': node_target[key],
            'priority': priority[key],
            'predecessor_step_ids': preds,
        })
    return InterpretationResult(ok=True, steps=steps, notes=notes)

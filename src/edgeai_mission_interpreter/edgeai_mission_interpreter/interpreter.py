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
   "T1 在 T2 之后救援", "T2 要等 T3 完成后", "X wait for Y"), a priority
   statement, a no-order statement ("in any order", "不分先后"), an
   exclusion ("不要救援 T2"), or a group of targets to rescue, optionally led
   by an order marker ("first", "then", "再", "最后").
3. A group led by "then"-like markers follows the previous group; a group
   after a condition follows the condition's targets; clauses in the same
   sentence without a marker join the current group.
4. No guessing: every rule records which characters of the instruction it
   used. Words that express order, conditions, priority or scope
   (``_CONSTRAINT_WORDS``) must all have been used by some rule; if one is
   left over, the interpreter does not know what it means and fails with 13
   instead of returning a plan that may be wrong. A no-order statement is
   also checked against the plan: targets said to have no order must not
   end up ordered.
5. The four unsettled cases (or-dependency, time limit, named robot, same
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
INTERPRETER_VERSION = '0.2.0'


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
    r'那个目标|这个目标|该目标|某个目标|(?<!这)一个(?:普通|协同)?目标|另一个|另外一个|最近|最远'
    r'|\b(?:closest|nearest|farthest)\b'
    r'|\b(?:a|another|some|that|this|the\s+other|the)\s+target\b', re.I)
_MISSION_WORDS = re.compile(r'救|目标|\brescue|\bsave\b|\btargets?\b', re.I)

# The "no guessing" lexicon: words that express order, conditions, priority or
# scope. Each occurrence must be used by some rule, otherwise the instruction
# fails with 13. Words that only describe robots or the action are not here.
_CONSTRAINT_WORDS = re.compile(
    # order
    r'\b(?:first(?:ly)?|initially|then|after(?:wards?)?|before(?:hand)?|next|finally'
    r'|last(?:ly)?|later|earlier|once|when(?:ever)?|while|until|till|wait\w*|prior'
    r'|following|followed|subsequent\w*|previous\w*|order\w*|sequen\w*|parallel'
    r'|simultaneous\w*|concurrent\w*|meanwhile|ahead|preced\w*|begin\w*|start\w*|between'
    r'|soon\w*|immediately|asap)\b'
    # conditions
    r'|\b(?:if|unless|provided|otherwise|instead|depend\w*|requir\w*|condition\w*)\b'
    # priority
    r'|\b(?:priorit\w*|urgen\w*|important|importance|critical|crucial|high(?:er|est)?'
    r'|low(?:er|est)?|most|least|prefer\w*|emergenc\w*)\b'
    # scope
    r'|\b(?:all|every|each|only|just|except|excluding|other|others|rest|remaining|both'
    r'|either|neither|any|none|nothing|these|those|them|they|or)\b'
    # Chinese: order, conditions, priority, scope
    r'|先|后|前(?!往|进)|再|接着|随即|继而|晚|早|依次|顺序|次序|同时|并行|等|直到|才|之间|最'
    r'|立即|立刻|马上|尽快'
    r'|如果|假如|要是|只有|除非|否则|前提|一旦|只要'
    r'|优先|紧急|急|重要|要紧'
    r'|所有|全部|全都|都|只|仅|除|其他|其余|其它|每|任意|任何|任一|或'
    r'|[两三四五六七八九十]个|两者|二者|三者|这些|它们|别的|其中', re.I)

_NAMED_ROBOT = re.compile(
    r'\brobot[_\s]?\d+\b|机器人\s*[_#]?\s*\d+|\d+\s*号\s*机器人', re.I)
# How many robots: the plan has no robot field, the coordinator decides.
_ROBOT_COUNT = re.compile(
    r'(?:两|二|一|单|多|\d)\s*台机器人|单机'
    r'|\b(?:both|two|\d|a\s+single|one|single|multiple)\s+robots?\b', re.I)
_TIME_LIMIT = re.compile(
    r'(?:在\s*)?\d+(?:\.\d+)?\s*(?:分钟|分|秒钟|秒|小时)\s*(?:之内|以内|内)'
    r'|\bwithin\s+\d+(?:\.\d+)?\s*(?:minutes?|mins?|seconds?|secs?|hours?)\b'
    r'|\bin\s+(?:under|less\s+than)\s+\d+(?:\.\d+)?\s*(?:minutes?|mins?|seconds?|secs?|hours?)\b'
    r'|\btime\s+limit\b|限时', re.I)
_OR = re.compile(r'(?:中)?任意一个|任一|或者|或|\bor\b|\beither\b|\bany\s+one\s+of\b'
                 r'|\bwhichever\b', re.I)
_TWICE = re.compile(r'\btwice\b|两次|二次|2\s*次', re.I)
_REPEAT = re.compile(r'再\S{0,6}一次|再次|又|\bagain\b|\bonce\s+more\b|\ba\s+second\s+time\b',
                     re.I)

_EXCLUDE = re.compile(
    r'除了|除|\bexcept(?:\s+for)?\b|\bother\s+than\b|\bbut\s+not\b|\bexcluding\b'
    r'|不要(?!求)|不用|不必|无需|\bdo\s+not\b|\bdon\'t\b|\bnever\b|\bskip\b', re.I)
_EXCLUDE_TAIL = re.compile(r'(?:以外|之外)')

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

_LEAD_FIRST = re.compile(
    r'^(?:请|让|麻烦)?\s*(?:首先|先)'
    r'|^(?:first(?:ly)?|first\s+of\s+all|initially|to\s+begin(?:\s+with)?)\b', re.I)
_LEAD_THEN = re.compile(
    r'^(?:请|让)?\s*(?:(?:然后|接着|随后|最后|之后|而后|接下来)\s*再?|再)'
    r'|^(?:and\s+)?(?:then|finally|afterwards?|after\s+that|next|lastly)\b', re.I)
_MID_THEN = re.compile(r'\s*(?=然后|接着|随后|最后)|\s+(?=(?:and\s+)?then\b)', re.I)
_FIRST_INLINE = re.compile(r'\bfirst\b|最先')
_LAST_INLINE = re.compile(r'\blast\b', re.I)
_ALSO = re.compile(r'也(?:要|需要|得|必须)|\balso\b|\bas\s+well\b(?!\s+as)', re.I)

_CHAIN = re.compile(
    r'依次|顺序(?:是|为)|按(?:照)?[^,]{0,40}?顺序'
    r'|\bone\s+(?:by|after)\s+(?:one|another)\b'
    r'|\bin\s+(?:exactly\s+)?(?:this|that|the\s+following|the\s+given)\s+order\b'
    r'|\bin\s+sequence\b|\bsequentially\b', re.I)
_CHAIN_WORDS = re.compile(
    r'依次(?:进行)?|的?顺序(?:是|为)?|按照?'
    r'|\bone\s+(?:by|after)\s+(?:one|another)\b'
    r'|\bin\s+(?:exactly\s+)?(?:this|that|the\s+following|the\s+given)\s+order\b'
    r'|\bin\s+sequence\b|\bsequentially\b', re.I)

_NO_ORDER = re.compile(
    r'(?:(?:两者|三者)?之间\s*)?(?:可以|都)?\s*(?:不分先后(?:顺序)?|顺序随意|先后(?:顺序)?都(?:可以|行)'
    r'|谁先谁后都(?:行|可以)|没有先后(?:要求|顺序)?|不要求(?:先后)?顺序|顺序不限'
    r'|没有顺序要求|同时(?:进行)?|并行)'
    r'|(?:\bcan\s+be\s+(?:rescued|saved)\s+)?\b(?:in\s+any\s+order|in\s+no\s+particular\s+order'
    r'|in\s+parallel|simultaneously|at\s+the\s+same\s+time)\b'
    r'|\b(?:the\s+)?order\s+does\s+not\s+matter\b|\border\s+is\s+not\s+(?:required|important)\b'
    r'|\bno\s+(?:required\s+)?order(?:\s+is\s+(?:required|needed))?\b', re.I)
_BETWEEN = re.compile(r'^\s*between\b', re.I)

_PRIORITY_NONE = re.compile(
    r'没有优先级(?:偏好|要求|区别|之分)?|无优先级|不分优先(?:级)?|优先级(?:相同|一样)'
    r'|同等优先(?:级)?'
    r'|\bno\s+priority(?:\s+(?:preference|is\s+given))?\b|\b(?:the\s+)?same\s+priority\b'
    r'|\bequal\s+priority\b|\bwithout\s+priority\b', re.I)
_PRIORITY_HINT = re.compile(r'优先|紧急|急|重要|要紧|\bpriorit\w*|\burgen\w*|\bimportan\w*',
                            re.I)
_PRIORITY_RANKING = re.compile(
    r'(?:按)?优先级(?:从高到低|由高到低)(?:依次)?(?:是|为)?\s*:?'
    r'|\bpriority\s+(?:from\s+)?high(?:est)?\s+to\s+low(?:est)?\s*:?'
    r'|\bin\s+(?:descending\s+)?order\s+of\s+priority\s*:?', re.I)
_PRIORITY_LEVELS = (
    ('top', re.compile(
        r'优先级最高|最高优先级|最紧急|最重要|最优先|最急|优先救援'
        r'|(?:\b(?:the|a)\s+)?\bhighest\s+priority\b|\bmost\s+(?:urgent|important)\b'
        r'|\btop\s+priority\b|\bfirst\s+priority\b', re.I)),
    ('bottom', re.compile(
        r'优先级最低|最低优先级|最不重要|最不紧急'
        r'|(?:\bthe\s+)?\blowest\s+priority\b|\bleast\s+(?:urgent|important)\b', re.I)),
    ('high', re.compile(
        r'优先级较高|优先级更高|更紧急|更重要|(?:\ba\s+)?\bhigher\s+priority\b'
        r'|\bmore\s+(?:urgent|important)\b', re.I)),
    ('low', re.compile(
        r'优先级较低|优先级更低|没那么急|不那么急|不太急|没那么紧急|没那么重要|不太重要'
        r'|(?:\ba\s+)?\blower\s+priority\b|\bless\s+(?:urgent|important)\b', re.I)),
    ('equal', re.compile(
        r'一样重要|同样重要|同等重要|一样紧急|同样紧急|\bequally\s+(?:important|urgent)\b', re.I)),
)
_PRIORITY_THEN = re.compile(r'^\s*(?:then|next|followed\s+by|然后|其次|接着|次之)', re.I)
_PRIORITY_AMONG = re.compile(r'其中')
_CONNECTOR_ONLY = re.compile(r'\s*(?:和|与|以及|及|、|,|\band\b|\bas\s+well\s+as\b)\s*', re.I)

_RESCUE_VERB = re.compile(r'救|\brescue\b|\bsave\b', re.I)

_ONLY = re.compile(r'(?:只需要|只需|只|仅需要|仅需|仅)|\b(?:only|just)\b', re.I)
_ONLY_TAIL = re.compile(
    r'这(?:一|两|三|几)个目标|这些目标|\bnothing\s+else\b|\bno\s+other\s+targets?\b'
    r'|别的(?:都)?不(?:用|要)救(?:援)?', re.I)
_BOTH_BEFORE_TARGETS = re.compile(r'\bboth\b(?=\s+TC?\d)', re.I)
_COUNT_AFTER_TARGETS = re.compile(r'(?<=\d)\s*[两三四五六七八九十]个(?:目标)?')
_ALL_LISTED = re.compile(r'(?:全都|都|全部)(?:要|需要|得|必须)?')
_LISTED_END = re.compile(r'(?:TC?\d+|目标|[两三四五六七八九十]个(?:目标)?)\s*$')
_LIST_JOIN_BLOCK = re.compile(
    r'\b(?:after|before|then|next|wait\w*|once|when|until)\b|后|前|等|在', re.I)

# Conditions and relations.
_LEADING_AND = re.compile(r'^\s*(?:and|but)\s+', re.I)
_EN_COND = re.compile(r'^\s*(?:only\s+)?(?:after|once|when)\b', re.I)
_EN_BEFORE_LEAD = re.compile(r'^\s*before\b(?:\s+(?:you|we)\s+(?:go\s+(?:and\s+)?)?'
                             r'(?:rescue|save)\b)?', re.I)
_EN_DONE = re.compile(
    r'\b(?:is|are|has\s+been|have\s+been)\s+(?:all\s+)?(?:completed|done|finished|rescued|saved)\b'
    r'|\b(?:completed|done|finished|rescued)\b', re.I)
_EN_ACTION_START = re.compile(r'\b(?:rescue|save|use|go)\b', re.I)
_EN_AFTER_MID = re.compile(r'(?:\bonly\s+)?\bafter\b', re.I)
_EN_BEFORE_MID = re.compile(r'\bbefore\b(?:\s+(?:you|we)\s+(?:go\s+(?:and\s+)?)?'
                            r'(?:rescue|save)\b)?', re.I)
_EN_WAIT = re.compile(
    r'(?:\bboth\s+)?(?:(?:have|has|need|needs)\s+to\s+|must\s+|should\s+|will\s+|shall\s+)?'
    r'\bwait(?:s)?\s+(?:for|until)\b', re.I)
_ZH_AFTER = re.compile(
    r'(?:都|全部|均)?\s*(?:已经?)?\s*(?:被)?(?:救援|营救)?(?:完成|结束|救完|做完)?\s*(?:了)?\s*'
    r'(?:之后|以后|(?<![最然随而先谁])后)\s*(?:才能?|再)?')
_ZH_DONE_END = re.compile(
    r'(?:都|全部|均)?\s*(?:已经?)?\s*(?:被)?(?:救援|营救)?(?:完成|结束|救完|做完)\s*(?:了)?\s*$')
_ZH_WAIT = re.compile(
    r'(?:要|需要|必须|得|应该)?\s*等(?:待|到)?(?P<y>.+?)'
    r'(?P<done>(?:都|全部|均)?\s*(?:已经?)?\s*(?:被)?(?:救援|营救)?(?:完成|结束|救完|做完)'
    r'\s*(?:了)?\s*(?:之后|以后|后)?\s*(?:才能?|再)?)')
_ZH_PREMISE = re.compile(
    r'的?前提(?:条件)?(?:是|为)(?P<y>.+?)'
    r'(?P<done>(?:都|全部|均)?\s*(?:已经?)?\s*(?:完成|结束|救完|做完)\s*(?:了)?)\s*$')
_ZH_ZAI = re.compile(
    r'(?:(?:都|全都)?\s*(?:要|需要|必须|得)?)\s*在(?P<y>[^在]+?)'
    r'(?P<mk>(?:完成|救完)?\s*(?P<dir>之后|以后|后|之前|以前))\s*(?:才能?|再)?')
_ZH_BEFORE = re.compile(r'(?:之前|以前)')
_ZH_FIRST_IN_REST = re.compile(r'^\s*(?:必须|要|需要|得)?\s*先')
_REFERENCE = re.compile(
    r'两者|二者|三者|它们|这些|这(?:两|三|四)个|(?:两|三|四)个|以上|上述|前面(?:的)?|全部|都'
    r'|\ball\s+(?:three|four|five|of\s+them|these|those)\b|\bboth(?:\s+of\s+them)?\b'
    r'|\bthem\b|\bthese(?:\s+(?:two|three|four))?\b|\bthose(?:\s+(?:two|three|four))?\b'
    r'|\bthey\b|\ball\b', re.I)


def normalize(text: str) -> str:
    """NFKC (full-width to ASCII), upper-case target ids, collapse whitespace."""
    text = unicodedata.normalize('NFKC', text or '')
    text = _TARGET.sub(lambda m: 'T' + m.group(1).upper() + m.group(2), text)
    return re.sub(r'\s+', ' ', text).strip()


class _Seg:
    """A slice of the normalized instruction that remembers where it starts."""

    __slots__ = ('text', 'off')

    def __init__(self, text: str, off: int):
        self.text = text
        self.off = off

    def sub(self, start: int, end: int | None = None) -> '_Seg':
        end = len(self.text) if end is None else end
        return _Seg(self.text[start:end], self.off + start)

    def strip(self, chars: str = ' ,') -> '_Seg':
        start, end = 0, len(self.text)
        while start < end and self.text[start] in chars:
            start += 1
        while end > start and self.text[end - 1] in chars:
            end -= 1
        return self.sub(start, end)

    def blank(self, start: int, end: int) -> '_Seg':
        return _Seg(self.text[:start] + ' ' * (end - start) + self.text[end:], self.off)

    def empty(self) -> bool:
        return not self.text.strip(' ,')


def _split(seg: _Seg, pattern: re.Pattern) -> list[_Seg]:
    out, start = [], 0
    for m in pattern.finditer(seg.text):
        out.append(seg.sub(start, m.start()))
        start = m.end()
    out.append(seg.sub(start))
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
        self.priority_marks: list[tuple[str, str]] = []   # (target, level)
        self.no_order: list[list[str]] = []         # node keys said to have no order
        self.last_keys: list[str] = []              # "rescue X last"
        self.used: list[tuple[int, int]] = []       # character spans used by a rule
        self.group_id = 0
        self.prev_group: list[str] = []             # keys of the latest action group
        self.last_mentioned: list[str] = []         # keys of the latest group or condition
        self.carry_preds: list[str] | None = None
        self._no_order_pending = False

    # bookkeeping -------------------------------------------------------------

    def _use(self, seg: _Seg, start: int, end: int):
        if end > start:
            self.used.append((seg.off + start, seg.off + end))

    def _use_all(self, seg: _Seg, pattern: re.Pattern):
        for m in pattern.finditer(seg.text):
            self._use(seg, m.start(), m.end())

    def unused_words(self, text: str) -> list[str]:
        covered = bytearray(len(text))
        for a, b in self.used:
            covered[a:b] = b'\x01' * (b - a)
        return [m.group(0) for m in _CONSTRAINT_WORDS.finditer(text)
                if not all(covered[m.start():m.end()])]

    # nodes -------------------------------------------------------------------

    def _ref(self, target: str) -> str:
        if target not in self.latest:
            self.latest[target] = target
            self.node_target[target] = target
        return self.latest[target]

    def _action(self, target: str, group: int, repeat: bool = False) -> str:
        if repeat and target in self.action_targets and self.action_group.get(target) != group:
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
                   group: int | None = None, repeat: bool = False) -> list[str]:
        group = self._new_group() if group is None else group
        keys = []
        for t in targets:
            key = self._action(t, group, repeat)
            for p in preds:
                self._edge(p, key)
            keys.append(key)
        return keys

    def _subjects(self, targets: Sequence[str]) -> list[str]:
        """Targets named as the subject of a relation: a new step, or the step already named."""
        group = self._new_group()
        return [self._action(t, group) for t in targets]

    # selection ---------------------------------------------------------------

    def _select(self, seg: _Seg, use: bool = False) -> list[str]:
        """Explicit target ids plus registry selections, in order of appearance."""
        found: list[tuple[int, str]] = []
        for m in _TARGET.finditer(seg.text):
            found.append((m.start(), 'T' + m.group(1).upper() + m.group(2)))
        taken: list[tuple[int, int]] = []
        for pattern, kind in ((_SELECT_NORMAL, NORMAL), (_SELECT_COOPERATIVE, COOPERATIVE),
                              (_SELECT_ALL, None)):
            for m in pattern.finditer(seg.text):
                if any(a <= m.start() < b for a, b in taken):
                    continue
                taken.append((m.start(), m.end()))
                if use:
                    self._use(seg, m.start(), m.end())
                if not self.registry:
                    raise _GiveUp(CODE_INSTRUCTION_AMBIGUOUS,
                                  '指令按类别选目标，但没有目标清单')
                for tid, ttype in self.registry.items():
                    if kind is None or ttype == kind:
                        found.append((m.start(), tid))
        if use:
            self._use_all(seg, _BOTH_BEFORE_TARGETS)
            self._use_all(seg, _COUNT_AFTER_TARGETS)
        out: list[str] = []
        for _, tid in sorted(found, key=lambda x: x[0]):
            if tid not in out:
                out.append(tid)
        return out

    def _refs(self, seg: _Seg) -> list[str]:
        """Targets of a condition: named ones, or a reference to the latest group."""
        targets = self._select(seg, use=True)
        if targets:
            return [self._ref(t) for t in targets]
        m = _REFERENCE.search(seg.text)
        if m and self.last_mentioned:
            self._use_all(seg, _REFERENCE)
            return list(self.last_mentioned)
        raise _GiveUp(CODE_INSTRUCTION_AMBIGUOUS, '前置条件没有指明是哪些目标')

    def _condition(self, seg: _Seg) -> list[str]:
        if _OR.search(seg.text):
            operands = self._select(seg)
            if len(operands) < 2:
                raise _GiveUp(CODE_INSTRUCTION_AMBIGUOUS, '「或」前置没有指明目标')
            self._use_all(seg, _OR)
            self.or_operands.append(operands)
        self._use_all(seg, _EN_DONE)
        preds = self._refs(seg)
        self.last_mentioned = preds
        return preds

    def _consume_group_words(self, seg: _Seg, sentence_has_only: bool):
        """Scope words that only restate the listed targets."""
        for m in _ALL_LISTED.finditer(seg.text):
            if _LISTED_END.search(seg.text[:m.start()]):
                self._use(seg, m.start(), m.end())
        if _TARGET.search(seg.text):
            self._use_all(seg, _ONLY)
            self._use_all(seg, _ONLY_TAIL)
        elif sentence_has_only:
            self._use_all(seg, _ONLY_TAIL)

    # sentences and clauses ---------------------------------------------------

    def parse(self, text: str):
        whole = _Seg(text, 0)
        self._use_all(whole, _ROBOT_COUNT)
        for sentence in _split(whole, _SENTENCE_SPLIT):
            sentence = sentence.strip()
            if not sentence.empty():
                self._sentence(sentence)
        if self.carry_preds and self.prev_group:
            for p in self.carry_preds:
                for k in self.prev_group:
                    self._edge(p, k)

    def _clauses(self, sentence: _Seg) -> list[_Seg]:
        parts: list[_Seg] = []
        for part in _split(sentence, re.compile(',')):
            parts += [c.strip() for c in _split(part, _MID_THEN) if not c.empty()]
        joined: list[_Seg] = []
        for c in parts:
            if joined:
                prev = joined[-1]
                list_continues = (re.search(r'TC?\d+\s*$', prev.text)
                                  and re.match(r'\s*TC?\d+', c.text)
                                  and not _LIST_JOIN_BLOCK.search(c.text)
                                  and not _PRIORITY_HINT.search(c.text)
                                  and not _RESCUE_VERB.search(c.text))
                if list_continues or _BETWEEN.match(prev.text):
                    joined[-1] = sentence.sub(prev.off - sentence.off,
                                              c.off - sentence.off + len(c.text))
                    continue
            joined.append(c)
        return joined

    def _sentence(self, sentence: _Seg):
        clauses = self._clauses(sentence)
        if _CHAIN.search(sentence.text) and not _OR.search(sentence.text) \
                and not _NO_ORDER.search(sentence.text):
            self._chain(sentence, clauses)
            return
        has_only = bool(_ONLY.search(sentence.text))
        current: list[str] = []          # keys of the group this sentence is building
        current_id: int | None = None
        current_preds: list[str] = []
        pending_preds: list[str] | None = self.carry_preds
        self.carry_preds = None
        pending_succ: list[str] = []
        pending_link = False
        in_priority = False
        led_first = False                # the current group was introduced by "first"
        for clause in clauses:
            body = self._strip_time_limit(clause)
            m = _LEADING_AND.match(body.text)
            if m:
                body = body.sub(m.end())

            m = _EXCLUDE.search(body.text)
            if m:
                self._use(body, m.start(), m.end())
                rest = body.sub(m.end())
                self._use_all(rest, _EXCLUDE_TAIL)
                self.excluded.update(self._select(rest, use=True))
                body = body.sub(0, m.start()).strip()
                if body.empty():
                    continue

            m = _PRIORITY_NONE.search(body.text)
            if m:
                self._use(body, m.start(), m.end())
                body = body.blank(m.start(), m.end())
                if body.empty():
                    continue

            m = _NO_ORDER.search(body.text)
            if m:
                self._use(body, m.start(), m.end())
                body = body.blank(m.start(), m.end())
                self._use_all(body, re.compile(r'^\s*between\b|之间', re.I))
                scope_targets = self._select(body, use=True)
                if scope_targets and all(t in self.latest for t in scope_targets):
                    self.no_order.append([self._ref(t) for t in scope_targets])
                    if not _RESCUE_VERB.search(body.text):
                        self._consume_group_words(body, has_only)
                        continue
                elif not scope_targets:
                    ref = _REFERENCE.search(body.text)
                    if ref and self.last_mentioned:
                        self._use_all(body, _REFERENCE)
                        scope = self.last_mentioned
                    else:
                        scope = current or self.prev_group
                    self.no_order.append(list(scope))
                    continue
                else:
                    # New targets: a group whose members have no order.
                    self._no_order_pending = True

            if _PRIORITY_HINT.search(body.text) or _PRIORITY_RANKING.search(body.text) or (
                    in_priority and not _RESCUE_VERB.search(body.text)
                    and self._select(body)):
                in_priority = self._priority(body, in_priority)
                continue

            m = _ALSO.search(body.text)
            if m and not _LIST_JOIN_BLOCK.search(body.text):
                targets = self._select(body, use=True)
                if targets:
                    self._use(body, m.start(), m.end())
                    for t in targets:
                        self._ref(t)
                        self.action_targets.add(t)
                    continue

            lead_then = _LEAD_THEN.match(body.text)
            lead_first = None if lead_then else _LEAD_FIRST.match(body.text)
            if lead_then:
                self._use(body, lead_then.start(), lead_then.end())
                body = body.blank(lead_then.start(), lead_then.end())
            elif lead_first and (self.group_id == 0 or pending_succ):
                self._use(body, lead_first.start(), lead_first.end())
                body = body.blank(lead_first.start(), lead_first.end())

            relation = self._relation(body)
            if relation is not None:
                kind, subj, other = relation
                if kind == 'cond':
                    preds = self._condition(other)
                    targets = self._select(subj, use=True)
                    if targets:
                        self._consume_group_words(subj, has_only)
                        current_id = self._new_group()
                        current = self._add_group(targets, preds, current_id)
                        current_preds = preds
                        self.prev_group = self.last_mentioned = current
                        pending_preds = None
                    else:
                        pending_preds = preds
                    continue
                if kind == 'before_lead':
                    succ = [self._ref(t) for t in self._select(other, use=True)]
                    targets = self._select(subj, use=True)
                    if targets:
                        keys = self._add_group(targets, [])
                        for k in keys:
                            for s in succ:
                                self._edge(k, s)
                        self.prev_group = self.last_mentioned = keys
                    else:
                        pending_succ = succ
                    continue
                if kind in ('after', 'before'):
                    targets = self._select(subj, use=True)
                    if targets:
                        self._consume_group_words(subj, has_only)
                        keys = self._subjects(targets)
                    elif current:
                        keys = list(current)
                    else:
                        raise _GiveUp(CODE_INSTRUCTION_AMBIGUOUS, '先后关系没有指明主语')
                    if kind == 'after':
                        for p in self._condition(other):
                            for k in keys:
                                self._edge(p, k)
                    else:
                        others = [self._ref(t) for t in self._select(other, use=True)]
                        if not others:
                            raise _GiveUp(CODE_INSTRUCTION_AMBIGUOUS, '先后关系没有指明目标')
                        for k in keys:
                            for s in others:
                                self._edge(k, s)
                    if not current:
                        current, current_preds = keys, []
                    self.prev_group = self.last_mentioned = keys
                    continue

            targets = self._select(body, use=True)
            if not targets:
                if lead_then:
                    pending_link = True
                elif has_only:
                    self._use_all(body, _ONLY_TAIL)
                continue
            if _OR.search(body.text):
                raise _GiveUp(CODE_INSTRUCTION_AMBIGUOUS, '「或」选目标，没说清救哪个')
            self._consume_group_words(body, has_only)
            if self.group_id == 0:
                self._use_all(body, _FIRST_INLINE)
            repeat = bool(_REPEAT.search(clause.text)) or bool(_TWICE.search(body.text))
            last = _LAST_INLINE.search(body.text)
            if pending_preds is not None:
                preds, pending_preds = pending_preds, None
            elif lead_then or pending_link:
                preds = list(self.prev_group)
            elif current_id is not None and not lead_first:
                if led_first:
                    # "先救援 T1，T2 和 T3 同时救援": same group, or the next step? Do not guess.
                    raise _GiveUp(CODE_INSTRUCTION_AMBIGUOUS, '「先」之后的分句没有说明先后')
                keys = self._add_group(targets, current_preds, current_id, repeat)
                current += [k for k in keys if k not in current]
                self.prev_group = self.last_mentioned = current
                continue
            else:
                preds = []
            pending_link = False
            led_first = bool(lead_first)
            current_id = self._new_group()
            current = self._add_group(targets, preds, current_id, repeat)
            current_preds = preds
            for k in current:
                for s in pending_succ:
                    self._edge(k, s)
            pending_succ = []
            if _TWICE.search(body.text):
                self._use_all(body, _TWICE)
                current_id = self._new_group()
                current = self._add_group(targets, current, current_id, repeat=True)
            if last:
                self._use(body, last.start(), last.end())
                self.last_keys += current
            if self._no_order_pending:
                self.no_order.append(list(current))
                self._no_order_pending = False
            self.prev_group = self.last_mentioned = current
        if pending_preds is not None:
            if current:
                # "Rescue X, after Y is done": a trailing condition belongs to X.
                for p in pending_preds:
                    for k in current:
                        self._edge(p, k)
            else:
                self.carry_preds = pending_preds

    def _chain(self, sentence: _Seg, clauses: Sequence[_Seg]):
        self._use_all(sentence, _CHAIN_WORDS)
        link = bool(clauses) and _LEAD_THEN.match(clauses[0].text)
        if link:
            self._use(clauses[0], link.start(), link.end())
        prev = list(self.prev_group) if link else []
        keys: list[str] = []
        for t in self._select(sentence, use=True):
            prev = self._add_group([t], prev)
            keys += prev
        self.prev_group = prev
        self.last_mentioned = keys

    def _relation(self, body: _Seg) -> tuple[str, _Seg, _Seg] | None:
        """
        Find an order relation in one clause.

        Returns (kind, subject, other):
        - 'cond': ``other`` names the condition, ``subject`` the targets to rescue after it
          (may be empty: the condition then applies to the next group);
        - 'before_lead': ``other`` must come after ``subject`` (subject may be empty);
        - 'after': ``subject`` comes after ``other``;
        - 'before': ``subject`` comes before ``other``.
        Every marker word the relation is built on is recorded as used.
        """
        text = body.text
        m = _EN_COND.match(text)
        if m:
            self._use(body, m.start(), m.end())
            rest = body.sub(m.end())
            done = _EN_DONE.search(rest.text)
            if done:
                self._use(rest, done.start(), done.end())
                return 'cond', rest.sub(done.end()), rest.sub(0, done.start())
            act = _EN_ACTION_START.search(rest.text)
            if act:
                return 'cond', rest.sub(act.start()), rest.sub(0, act.start())
            return 'cond', rest.sub(len(rest.text)), rest
        m = _EN_BEFORE_LEAD.match(text)
        if m:
            self._use(body, m.start(), m.end())
            rest = body.sub(m.end())
            act = _EN_ACTION_START.search(rest.text)
            if act and self._select(rest.sub(0, act.start())):
                return 'before_lead', rest.sub(act.start()), rest.sub(0, act.start())
            return 'before_lead', rest.sub(len(rest.text)), rest
        m = _EN_WAIT.search(text)
        if m:
            self._use(body, m.start(), m.end())
            other = body.sub(m.end())
            self._use_all(other, re.compile(r'^\s*both\b', re.I))
            return 'after', body.sub(0, m.start()), other
        m = _EN_AFTER_MID.search(text)
        if m and self._select(body.sub(0, m.start())):
            self._use(body, m.start(), m.end())
            return 'after', body.sub(0, m.start()), body.sub(m.end())
        m = _EN_BEFORE_MID.search(text)
        if m and self._select(body.sub(0, m.start())):
            self._use(body, m.start(), m.end())
            return 'before', body.sub(0, m.start()), body.sub(m.end())
        m = _ZH_PREMISE.search(text)
        if m:
            self._use(body, m.start(), m.start('y'))
            self._use(body, m.start('done'), m.end('done'))
            return 'after', body.sub(0, m.start()), body.sub(m.start('y'), m.end('y'))
        m = _ZH_WAIT.search(text)
        if m:
            self._use(body, m.start(), m.start('y'))
            self._use(body, m.start('done'), m.end('done'))
            rest = body.sub(m.end())
            subj = rest if self._select(rest) else body.sub(0, m.start())
            return 'after', subj, body.sub(m.start('y'), m.end('y'))
        m = _ZH_ZAI.search(text)
        if m:
            subject = body.sub(0, m.start())
            other = body.sub(m.start('y'), m.end('y'))
            rest = body.sub(m.end())
            spans = [(m.start(), m.start('y')), (m.start('mk'), m.end())]
            after = m.group('dir') in ('之后', '以后', '后')
            if self._select(subject):
                for a, b in spans:
                    self._use(body, a, b)
                return ('after' if after else 'before'), subject, other
            if self._select(other):
                for a, b in spans:
                    self._use(body, a, b)
                if after:
                    return 'cond', rest, other
                fm = _ZH_FIRST_IN_REST.match(rest.text)
                if fm:
                    self._use(rest, fm.start(), fm.end())
                return 'before_lead', rest, other
        m = _ZH_BEFORE.search(text)
        if m:
            left, rest = body.sub(0, m.start()), body.sub(m.end())
            if self._select(left):
                self._use(body, m.start(), m.end())
                fm = _ZH_FIRST_IN_REST.match(rest.text)
                if self._select(rest):
                    if fm:
                        self._use(rest, fm.start(), fm.end())
                    return 'after', left, rest
                return 'before_lead', rest, left
        m = _ZH_AFTER.search(text)
        if m:
            left, right = body.sub(0, m.start()), body.sub(m.end())
            marker = body.sub(0, m.end())
            if self._select(left) or (_REFERENCE.search(marker.text) and self.last_mentioned):
                self._use(body, m.start(), m.end())
                cm = re.match(r'^\s*只有', left.text)
                if cm:
                    self._use(left, cm.start(), cm.end())
                if not self._select(left):
                    left = marker
                return 'cond', right, left
        m = _ZH_DONE_END.search(text)
        if m and m.start() > 0:
            left = body.sub(0, m.start())
            if self._select(left) or (_REFERENCE.search(text) and self.last_mentioned):
                self._use(body, m.start(), m.end())
                return 'cond', body.sub(len(text)), body.sub(0, m.end())
        return None

    def _priority(self, seg: _Seg, continuing: bool) -> bool:
        m = _PRIORITY_RANKING.search(seg.text)
        if m:
            self._use(seg, m.start(), m.end())
            targets = self._select(seg.sub(m.end()), use=True)
            if len(targets) < 2:
                raise _GiveUp(CODE_INSTRUCTION_AMBIGUOUS, '优先级排序没有列出目标')
            for i, t in enumerate(targets):
                self._ref(t)
                self.priority_marks.append((t, f'rank{i}'))
            return True
        self._use_all(seg, _PRIORITY_AMONG)
        occurrences = list(_TARGET.finditer(seg.text))
        levels: list[list[Any]] = []
        for i, occ in enumerate(occurrences):
            end = occurrences[i + 1].start() if i + 1 < len(occurrences) else len(seg.text)
            piece = seg.sub(occ.end(), end)
            level = None
            for name, pattern in _PRIORITY_LEVELS:
                lm = pattern.search(piece.text)
                if lm:
                    self._use(piece, lm.start(), lm.end())
                    level = name
                    break
            levels.append([occ.group(0), level, piece])
        for i in reversed(range(len(levels) - 1)):
            if levels[i][1] is None and _CONNECTOR_ONLY.fullmatch(levels[i][2].text):
                levels[i][1] = levels[i + 1][1]
        then = _PRIORITY_THEN.match(seg.text)
        for target, level, _ in levels:
            if level is None and continuing and then:
                self._use(seg, then.start(), then.end())
                level = 'next'
            if level is None:
                continue
            self._ref(target)
            self.priority_marks.append((target, level))
        return True

    def _strip_time_limit(self, clause: _Seg) -> _Seg:
        m = _TIME_LIMIT.search(clause.text)
        if not m:
            return clause
        targets = self._select(clause)
        self.time_limited.append((targets, m.group(0).strip()))
        self._use(clause, m.start(), m.end())
        return clause.blank(m.start(), m.end())


# ------------------------------------------------------------------- public API

def _fail(code: int, detail: str, notes: Sequence[str] = ()) -> InterpretationResult:
    return InterpretationResult(ok=False, issues=[Issue(code=code, detail=detail)],
                                notes=list(notes))


# Position of each priority level, highest first; unmentioned targets are 'equal'.
_LEVEL_POSITION = {'top': 0, 'high': 500, 'equal': 1000, 'low': 1500, 'bottom': 2000}


def _priorities(marks: Sequence[tuple[str, str]], targets: Sequence[str]) -> dict[str, int]:
    """Turn priority marks into numbers: the lowest tier present gets 0."""
    if not marks:
        return {}
    position: dict[str, int] = {}
    last = None
    for target, level in marks:
        if level.startswith('rank'):
            position[target] = int(level[4:])
            last = position[target]
        elif level == 'next':
            last = (last if last is not None else 0) + 1
            position[target] = last
        else:
            position[target] = _LEVEL_POSITION[level]
            last = position[target]
    for t in targets:
        position.setdefault(t, _LEVEL_POSITION['equal'])
    tiers = sorted({position[t] for t in targets}, reverse=True)
    return {t: tiers.index(position[t]) for t in targets}


def _reaches(edges: Sequence[tuple[str, str]], start: str, goal: str) -> bool:
    succ: dict[str, list[str]] = {}
    for a, b in edges:
        succ.setdefault(a, []).append(b)
    seen, stack = set(), [start]
    while stack:
        node = stack.pop()
        for nxt in succ.get(node, []):
            if nxt == goal:
                return True
            if nxt not in seen:
                seen.add(nxt)
                stack.append(nxt)
    return False


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
        # Blank the names with spaces of the same length so character positions stay put.
        text = _NAMED_ROBOT.sub(lambda m: ' ' * len(m.group(0)), text)
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

    unused = parser.unused_words(text)
    if unused:
        words = '」「'.join(dict.fromkeys(unused))
        return _fail(CODE_INSTRUCTION_AMBIGUOUS,
                     f'指令里的「{words}」没有被任何规则用上，不猜', notes)

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

    for key in parser.last_keys:
        for other in node_target:
            if other == key:
                continue
            if _reaches(edges, key, other):
                return _fail(CODE_INSTRUCTION_AMBIGUOUS, '「最后」和别的先后关系矛盾', notes)
            if not _reaches(edges, other, key):
                edges.append((other, key))

    for scope in parser.no_order:
        for a in scope:
            for b in scope:
                if a != b and _reaches(edges, a, b):
                    return _fail(CODE_INSTRUCTION_AMBIGUOUS,
                                 '说了不分先后的目标在计划里有先后，矛盾', notes)

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

    by_target = _priorities(parser.priority_marks, list(dict.fromkeys(node_target.values())))
    priority = {key: by_target.get(target, 0) for key, target in node_target.items()}
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

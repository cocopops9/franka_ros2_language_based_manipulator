// Faithful JavaScript port of hmi_pkg/nodes/nlu_engine.py and of the transcript
// post-processing in hmi_pkg/nodes/asr_whisper.py.
//
// The port keeps the original control flow, thresholds and quirks, so that the
// interactive documentation shows what the real server would return.
// docs/tools/check_parity.py compares this file against the Python modules.
//
// One unavoidable difference: when two candidates tie, Python picks the one that
// comes first in *set iteration order*, which depends on string hash
// randomisation (PYTHONHASHSEED). This port resolves ties by insertion order,
// which is one of the outcomes Python can produce.

// Copy of hmi_pkg/config/capabilities.json (checked by docs/tools/check_parity.py).
export const CAPABILITIES = {
  version: "1.1",
  actions: ["pick", "release", "next", "stop", "cancel"],
  objects: ["cup", "bottle", "mug", "can", "item"],
  standalone_actions: ["stop", "next", "cancel", "release"],
  object_required_actions: ["pick", "release"],
  synonyms: {
    actions: {
      pick: ["pick up", "grab", "take", "get", "hold", "lift"],
      release: ["drop", "let go", "open gripper", "drop it", "put down", "place"],
      next: ["next one", "grab next", "take next", "continue", "proceed"],
      stop: ["halt", "freeze", "pause", "wait", "hold on"],
      cancel: ["undo", "abort", "never mind", "nevermind", "forget it", "skip"],
    },
    objects: {
      cup: ["glass", "tumbler"],
      bottle: ["flask", "container"],
      mug: ["coffee cup", "tea cup"],
      can: ["tin", "aluminum can", "soda can"],
      item: ["object", "thing", "stuff"],
    },
  },
  phonetic_corrections: {
    actions: {
      pick: ["peak", "pig", "peek", "pik", "pic", "beak"],
      cancel: ["council", "cancell", "cansel", "cancle"],
      release: ["releaze", "relese", "relees", "reliefs"],
      stop: ["top", "spot", "stock"],
      next: ["necks", "nest", "text"],
    },
    objects: {
      can: ["ken", "kahn", "cann"],
      cup: ["cub", "cop", "cupp"],
      mug: ["mag", "mugg", "maug"],
      bottle: ["modal", "bottel", "bottled"],
      item: ["i tem", "it em", "aitem"],
    },
  },
};

// Hard-coded "soft" corrections from NLU.load(). Note that the
// "phonetic_corrections" block of capabilities.json is never read by the server:
// only these two tables are used.
const SOFT_ACTIONS = [
  ["pick", ["peak", "pig", "peek", "ike", "pik", "pic", "beak", "pick up", "grab", "take"]],
  ["cancel", ["council", "cancell", "cansel", "cancle", "cancelled", "counsel"]],
  ["release", ["releaze", "relese", "relees", "rel ease", "reliefs", "drop"]],
  ["stop", ["top", "spot", "stock"]],
  ["next", ["necks", "nest", "text"]],
];

const SOFT_OBJECTS = [
  ["can", ["ken", "cane", "aken", "kan", "a can"]],
  ["cup", ["cub", "cap", "kup"]],
  ["bottle", ["botto", "bottal", "batalla", "bataille", "a bottle", "bottol", "bottle."]],
  ["mug", ["mag", "mog", "mugg", "mug."]],
];

// Config.NLU_ACTION_THRESHOLD / Config.NLU_OBJECT_THRESHOLD (hmi_pkg/nodes/config.py).
// load_capabilities() overwrites the dataclass defaults (0.65 / 0.70) with these.
export const ACTION_THRESHOLD = 0.75;
export const OBJECT_THRESHOLD = 0.75;

// ---------------------------------------------------------------------------
// String metrics
// ---------------------------------------------------------------------------

export function editDistance(a, b, maxCutoff = 3) {
  const la = a.length;
  const lb = b.length;
  if (Math.abs(la - lb) > maxCutoff) return maxCutoff + 1;

  let prev = [];
  for (let j = 0; j <= lb; j++) prev.push(j);

  for (let i = 1; i <= la; i++) {
    const cur = [i];
    for (let j = 0; j < lb; j++) cur.push(0);
    let rowMin = maxCutoff + 1;
    const ca = a[i - 1];
    for (let j = 1; j <= lb; j++) {
      const cost = ca === b[j - 1] ? 0 : 1;
      cur[j] = Math.min(prev[j] + 1, cur[j - 1] + 1, prev[j - 1] + cost);
      if (cur[j] < rowMin) rowMin = cur[j];
    }
    if (rowMin > maxCutoff) return maxCutoff + 1;
    prev = cur;
  }
  return prev[prev.length - 1];
}

const PHONETIC_MAP = {
  c: "k", k: "c", s: "z", z: "s", f: "v", v: "f",
  b: "p", p: "b", d: "t", t: "d", g: "k", x: "ks",
};

function phoneticNormalize(word) {
  const w = word.split("ck").join("k").split("ph").join("f").split("gh").join("f");
  let out = "";
  for (const ch of w) out += PHONETIC_MAP[ch] !== undefined ? PHONETIC_MAP[ch] : ch;
  return out;
}

export function phoneticSimilarity(a, b) {
  a = a.toLowerCase();
  b = b.toLowerCase();
  if (a === b) return 1.0;

  const a2 = phoneticNormalize(a);
  const b2 = phoneticNormalize(b);
  if (a2 === b2) return 0.95;

  const dist = editDistance(a2, b2, 4);
  const maxLen = Math.max(a2.length, b2.length) || 1;
  let sim = 1.0 - dist / maxLen;

  if (a && b && a[0] === b[0]) sim += 0.1;

  const aC = a.replace(/[aeiou]/g, "");
  const bC = b.replace(/[aeiou]/g, "");
  if (aC && bC) {
    sim += (1.0 - editDistance(aC, bC, 3) / Math.max(aC.length, bC.length)) * 0.15;
  }
  return Math.min(1.0, sim);
}

// ---------------------------------------------------------------------------
// Phrase matching
// ---------------------------------------------------------------------------

// Python's re.escape (3.7+) escapes exactly these characters.
const PY_SPECIAL = new Set("()[]{}?*+-|^$\\.&~# \t\n\r\v\f".split(""));

function pythonReEscape(text) {
  let out = "";
  for (const ch of text) out += PY_SPECIAL.has(ch) ? "\\" + ch : ch;
  return out;
}

// Mirrors NLU._compile_phrase. re.escape turns "pick up" into "pick\ up", and the
// following substitution turns the escaped space into "\s+", which yields the
// regex  pick\\s+up : a literal backslash followed by one or more "s".
// Consequently, multi-word aliases can never match cleaned text.
const phraseCache = new Map();
function compilePhrase(phrase) {
  if (phraseCache.has(phrase)) return phraseCache.get(phrase);
  const escaped = pythonReEscape(phrase.trim().toLowerCase());
  const source = escaped.replace(/\s+/g, "\\s+");
  const regex = new RegExp("(?<!\\w)" + source + "(?!\\w)");
  phraseCache.set(phrase, regex);
  return regex;
}

function firstMatch(text, phrases) {
  // sorted(..., key=len, reverse=True) is stable: equal lengths keep input order.
  const ordered = [...phrases].sort((x, y) => y.length - x.length);
  for (const ph of ordered) {
    const m = compilePhrase(ph).exec(text);
    if (m) return { phrase: ph, start: m.index, end: m.index + m[0].length };
  }
  return null;
}

// Every phrase of `phrases` whose compiled pattern matches `text` (used by the lab to
// show ties that Python would break by set iteration order).
export function allPhraseMatches(text, phrases) {
  const out = [];
  for (const ph of phrases) {
    const m = compilePhrase(ph).exec(text);
    if (m) out.push({ phrase: ph, start: m.index, end: m.index + m[0].length });
  }
  return out;
}

// ---------------------------------------------------------------------------
// NLU model
// ---------------------------------------------------------------------------

export class NLU {
  constructor(capabilities = CAPABILITIES) {
    this.actions = new Set(capabilities.actions || []);
    this.objects = new Set(capabilities.objects || []);
    this.synAct = new Map();
    this.synObj = new Map();
    this.denyActionTokens = new Set(["please", "pls", "plz"]);
    this.actionThreshold = ACTION_THRESHOLD;
    this.objectThreshold = OBJECT_THRESHOLD;

    const syn = capabilities.synonyms || {};
    for (const a of capabilities.actions || []) this.synAct.set(a.toLowerCase(), a);
    for (const o of capabilities.objects || []) this.synObj.set(o.toLowerCase(), o);
    for (const [k, list] of Object.entries(syn.actions || {})) {
      for (const w of list) this.synAct.set(w.toLowerCase(), k);
    }
    for (const [k, list] of Object.entries(syn.objects || {})) {
      for (const w of list) this.synObj.set(w.toLowerCase(), k);
    }
    // dict.setdefault: only add the alias if it is not already defined.
    for (const [canon, aliases] of SOFT_ACTIONS) {
      for (const w of aliases) if (!this.synAct.has(w)) this.synAct.set(w, canon);
    }
    for (const [canon, aliases] of SOFT_OBJECTS) {
      for (const w of aliases) if (!this.synObj.has(w)) this.synObj.set(w, canon);
    }
  }

  fuzzyToAction(token, trace) {
    if (this.denyActionTokens.has(token)) return null;
    if (this.synAct.has(token)) {
      const cand = this.synAct.get(token);
      trace?.push({ step: "action-alias", token, alias: token, result: cand });
      return this.actions.has(cand) ? cand : null;
    }
    const keys = new Set([...this.synAct.keys(), ...[...this.actions].map((a) => a.toLowerCase())]);
    let bestEdit = [null, 999];
    let bestPhon = [null, 0.0];
    for (const k of keys) {
      const d = editDistance(token, k, 3);
      if (d < bestEdit[1]) bestEdit = [k, d];
      const sim = phoneticSimilarity(token, k);
      if (sim > bestPhon[1]) bestPhon = [k, sim];
    }
    const canon = (key) => (key ? (this.synAct.has(key) ? this.synAct.get(key) : key) : null);
    trace?.push({ step: "action-fuzzy", token, bestPhonetic: bestPhon, bestEdit, threshold: this.actionThreshold });

    if (bestPhon[1] >= this.actionThreshold) {
      const c = canon(bestPhon[0]);
      return this.actions.has(c) ? c : null;
    }
    if (bestEdit[0]) {
      const c = canon(bestEdit[0]);
      if (this.actions.has(c)) {
        if (bestEdit[1] <= 1) return c;
        if (c === "cancel" && bestEdit[1] <= 3) return c;
        if (bestEdit[1] <= 2) return c;
      }
    }
    return null;
  }

  fuzzyToObject(token, trace) {
    if (this.synObj.has(token)) {
      const cand = this.synObj.get(token);
      trace?.push({ step: "object-alias", token, alias: token, result: cand });
      return this.objects.has(cand) ? cand : null;
    }
    const keys = new Set([...this.synObj.keys(), ...[...this.objects].map((o) => o.toLowerCase())]);
    let bestEdit = [null, 999];
    let bestPhon = [null, 0.0];
    for (const k of keys) {
      const d = editDistance(token, k, 3);
      if (d < bestEdit[1]) bestEdit = [k, d];
      const sim = phoneticSimilarity(token, k);
      if (sim > bestPhon[1]) bestPhon = [k, sim];
    }
    const canon = (key) => (key ? (this.synObj.has(key) ? this.synObj.get(key) : key) : null);
    trace?.push({ step: "object-fuzzy", token, bestPhonetic: bestPhon, bestEdit, threshold: this.objectThreshold });

    if (bestPhon[1] >= this.objectThreshold) {
      const c = canon(bestPhon[0]);
      return this.objects.has(c) ? c : null;
    }
    if (bestEdit[0] && bestEdit[1] <= 2) {
      const c = canon(bestEdit[0]);
      return this.objects.has(c) ? c : null;
    }
    return null;
  }

  // Returns {action, object} or null, exactly like NLU.parse.
  // If `trace` is an array, the decision steps are appended to it.
  parse(text, trace) {
    const clean = text.toLowerCase().replace(/[^a-z0-9]+/g, " ").trim();
    if (!clean) return null;
    const tokens = clean.split(/\s+/);
    trace?.push({ step: "clean", clean, tokens });

    let action = null;
    const actMatch = firstMatch(clean, this.synAct.keys());
    if (actMatch) {
      const cand = this.synAct.has(actMatch.phrase) ? this.synAct.get(actMatch.phrase) : actMatch.phrase;
      trace?.push({ step: "action-phrase", phrase: actMatch.phrase, result: cand });
      if (this.actions.has(cand)) action = cand;
    }
    if (!action) {
      for (const t of tokens) {
        const ca = this.fuzzyToAction(t, trace);
        if (ca && this.actions.has(ca)) { action = ca; break; }
      }
    }

    let object = null;
    if (action === "pick") {
      const m = firstMatch(clean, ["pick", "pick up"]);
      const start = m ? m.end : 0;
      const post = clean.slice(start).trim();
      trace?.push({ step: "after-pick", post });
      const om = firstMatch(post, this.synObj.keys());
      if (om) {
        const co = this.synObj.has(om.phrase) ? this.synObj.get(om.phrase) : om.phrase;
        trace?.push({ step: "object-phrase", phrase: om.phrase, result: co });
        if (this.objects.has(co)) object = co;
      }
      if (!object && post) {
        for (const t of post.split(/\s+/)) {
          const co = this.fuzzyToObject(t, trace);
          if (co && this.objects.has(co)) { object = co; break; }
        }
      }
    }
    if (!object) {
      const om2 = firstMatch(clean, this.synObj.keys());
      if (om2) {
        const co = this.synObj.has(om2.phrase) ? this.synObj.get(om2.phrase) : om2.phrase;
        trace?.push({ step: "object-phrase", phrase: om2.phrase, result: co });
        if (this.objects.has(co)) object = co;
      }
    }
    if (!object) {
      for (const t of tokens) {
        const co = this.fuzzyToObject(t, trace);
        if (co && this.objects.has(co)) { object = co; break; }
      }
    }

    if (action || object) return { action, object };
    return null;
  }
}

// ---------------------------------------------------------------------------
// Transcript post-processing (asr_whisper.py)
// ---------------------------------------------------------------------------

const TRANSCRIPT_CORRECTIONS = [
  [/\bike\s+aken\b/gi, "pick can"],
  [/\bpeak\s+can\b/gi, "pick can"],
  [/\bpicca\s*bott?o\b/gi, "pick bottle"],
  [/\bpick\s+bott?al\b/gi, "pick bottle"],
  [/\bcouncil\b/gi, "cancel"],
  [/\bcounsel\b/gi, "cancel"],
  [/\brel\s+ease\b/gi, "release"],
];

export function postProcessTranscript(text) {
  if (!text) return text;
  let result = text;
  for (const [pattern, replacement] of TRANSCRIPT_CORRECTIONS) {
    result = result.replace(pattern, replacement);
  }
  return result;
}

// asr_whisper._english_like with EN_ONLY_STRICT=1: every character must be ASCII.
export function isEnglishLike(text) {
  for (const ch of text) if (ch.codePointAt(0) >= 128) return false;
  return true;
}

// state.norm_txt, used for de-duplication.
export function normText(text) {
  return text.toLowerCase().replace(/[^a-z0-9]+/g, " ").trim();
}

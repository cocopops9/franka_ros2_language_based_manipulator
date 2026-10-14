// Documentation viewer: renders the Markdown chapters of docs/ (the same files GitHub shows)
// and mounts an interactive lab wherever a chapter contains an HTML comment <!-- lab:name -->.

const REPO = "https://github.com/cocopops9/franka_ros2_language_based_manipulator";
const BLOB = REPO + "/blob/main/";
const SITE = "https://cocopops9.github.io/franka_ros2_language_based_manipulator/";

export const CHAPTERS = [
  { id: "README", n: "0", title: "Guided tour", sub: "how to read these docs" },
  { id: "01-system-overview", n: "1", title: "System overview", sub: "processes and messages" },
  { id: "02-mission-and-scheduling", n: "2", title: "Mission and scheduling", sub: "what to pick, and when" },
  { id: "03-perception", n: "3", title: "Perception", sub: "from pixels to 3-D points" },
  { id: "04-calibration-and-frames", n: "4", title: "Calibration, frames, memory", sub: "camera to robot base" },
  { id: "05-voice-interface", n: "5", title: "Voice interface", sub: "from speech to an intent" },
  { id: "06-motion-and-grasping", n: "6", title: "Motion and grasping", sub: "poses, paths, grasps" },
  { id: "07-end-to-end-run", n: "7", title: "End-to-end run", sub: "one mission, all failure modes" },
];

const LABS = {
  rosgraph: { title: "ROS graph explorer", sub: "click a node" },
  mission: { title: "Mission planner", sub: "controller scheduling rule" },
  camera: { title: "Virtual camera", sub: "detection, depth, deprojection" },
  frames: { title: "3-D frames", sub: "drag to rotate, scroll to zoom" },
  memory: { title: "Object memory", sub: "2 cm dead band" },
  vad: { title: "Voice activity detection", sub: "microphone or synthetic input" },
  nlu: { title: "NLU parser", sub: "port of nlu_engine.py" },
  intent: { title: "Intent and lock state machine", sub: "port of websocket_handler.py" },
  pickpose: { title: "Pick-pose calculator", sub: "offsets of move_to_pose.cpp" },
  cartesian: { title: "Cartesian path", sub: "planar illustration" },
  grasp: { title: "Grasp tolerance", sub: "libfranka success rule" },
  e2e: { title: "Mission simulator", sub: "the whole controller loop" },
};

const article = document.getElementById("article");
const steps = document.getElementById("steps");

// ---------------------------------------------------------------------------
// Preferences (per viewer, optional)
// ---------------------------------------------------------------------------

function readStore(key, fallback) {
  try {
    const v = localStorage.getItem(key);
    return v === null ? fallback : JSON.parse(v);
  } catch (e) {
    return fallback;
  }
}

function writeStore(key, value) {
  try {
    localStorage.setItem(key, JSON.stringify(value));
  } catch (e) {
    /* storage unavailable: preferences are simply not remembered */
  }
}

export function isDark() {
  const forced = document.documentElement.getAttribute("data-theme");
  if (forced) return forced === "dark";
  return window.matchMedia && window.matchMedia("(prefers-color-scheme: dark)").matches;
}

function applyTheme(theme) {
  if (theme) document.documentElement.setAttribute("data-theme", theme);
  else document.documentElement.removeAttribute("data-theme");
  document.getElementById("theme-btn").textContent = isDark() ? "Light" : "Dark";
}

applyTheme(readStore("dume-theme", null));
document.getElementById("theme-btn").addEventListener("click", () => {
  const next = isDark() ? "light" : "dark";
  writeStore("dume-theme", next);
  applyTheme(next);
  window.dispatchEvent(new CustomEvent("themechange"));
  route();
});

document.getElementById("menu-btn").addEventListener("click", () => document.body.classList.toggle("menu-open"));

// ---------------------------------------------------------------------------
// Sidebar
// ---------------------------------------------------------------------------

function renderSteps(activeId) {
  const visited = new Set(readStore("dume-visited", []));
  steps.innerHTML = "";
  for (const ch of CHAPTERS) {
    const li = document.createElement("li");
    const a = document.createElement("a");
    a.href = "#/" + ch.id;
    a.className = (ch.id === activeId ? "active " : "") + (visited.has(ch.id) ? "visited" : "");
    a.innerHTML = `<span class="dot">${ch.n}</span><span>${ch.title}<span class="sub">${ch.sub}</span></span>`;
    a.addEventListener("click", () => document.body.classList.remove("menu-open"));
    li.appendChild(a);
    steps.appendChild(li);
  }
}

// ---------------------------------------------------------------------------
// Markdown rendering
// ---------------------------------------------------------------------------

function decodeEntities(text) {
  const t = document.createElement("textarea");
  t.innerHTML = text;
  return t.value;
}

function renderMath(tex, display) {
  try {
    return window.katex.renderToString(tex, { displayMode: display, throwOnError: false, strict: "ignore" });
  } catch (e) {
    return `<code>${tex}</code>`;
  }
}

function slugify(text) {
  return text.toLowerCase().replace(/[^a-z0-9]+/g, "-").replace(/^-|-$/g, "");
}

function rewriteLink(a, currentId) {
  const href = a.getAttribute("href");
  if (!href) return;
  if (href.startsWith(SITE)) {
    a.setAttribute("href", href.slice(SITE.length) || "#/");
    return;
  }
  if (/^[a-z]+:/i.test(href)) {
    a.target = "_blank";
    a.rel = "noopener";
    return;
  }
  if (href.startsWith("#")) {
    const anchor = href.slice(1);
    a.addEventListener("click", (ev) => {
      ev.preventDefault();
      const el = document.getElementById(anchor);
      if (el) el.scrollIntoView({ behavior: "smooth" });
    });
    return;
  }
  const mdMatch = href.match(/^([A-Za-z0-9_-]+)\.md(#.*)?$/);
  if (mdMatch) {
    a.setAttribute("href", "#/" + mdMatch[1]);
    return;
  }
  if (href.startsWith("../")) {
    a.setAttribute("href", BLOB + href.replace(/^(\.\.\/)+/, ""));
    a.target = "_blank";
    a.rel = "noopener";
    return;
  }
  // Paths inside docs/ that are not chapters (tools/, assets/) live in the repository.
  a.setAttribute("href", BLOB + "docs/" + href.replace(/^\.\//, ""));
  a.target = "_blank";
  a.rel = "noopener";
}

async function mountLabs(root, wanted) {
  const walker = document.createTreeWalker(root, NodeFilter.SHOW_COMMENT);
  const found = [];
  while (walker.nextNode()) {
    const m = walker.currentNode.nodeValue.trim().match(/^lab:([a-z0-9]+)$/);
    if (m) found.push([walker.currentNode, m[1]]);
  }
  for (const [comment, name] of found) {
    const meta = LABS[name] || { title: name, sub: "" };
    const section = document.createElement("section");
    section.className = "lab";
    section.id = "lab-" + name;
    section.innerHTML = `<div class="lab-head"><span class="lab-badge">Lab</span><span class="lab-title">${meta.title}</span><span class="lab-sub">${meta.sub}</span></div><div class="lab-body"><p class="muted">Loading lab...</p></div>`;
    comment.parentNode.insertBefore(section, comment);
    // The blockquote that follows the marker only links to this page from GitHub.
    let next = section.nextSibling;
    while (next && next.nodeType !== 1) next = next.nextSibling;
    if (next && next.tagName === "BLOCKQUOTE" && /Interactive lab/.test(next.textContent)) next.remove();
    const body = section.querySelector(".lab-body");
    try {
      const mod = await import(`./labs/${name}.js`);
      body.innerHTML = "";
      mod.mount(body);
      if (mod.foot) {
        const foot = document.createElement("div");
        foot.className = "lab-foot";
        foot.textContent = mod.foot;
        section.appendChild(foot);
      }
    } catch (err) {
      body.innerHTML = `<p class="error-box">This lab could not be loaded: ${String(err.message || err)}</p>`;
      console.error(err);
    }
  }
  if (wanted) {
    const el = document.getElementById("lab-" + wanted);
    if (el) el.scrollIntoView({ behavior: "auto", block: "start" });
  }
}

function addPager(currentId) {
  const i = CHAPTERS.findIndex((c) => c.id === currentId);
  const pager = document.createElement("nav");
  pager.className = "pager";
  if (i > 0) {
    const p = CHAPTERS[i - 1];
    pager.innerHTML += `<a class="prev" href="#/${p.id}"><span class="label">Previous</span>${p.n}. ${p.title}</a>`;
  }
  if (i < CHAPTERS.length - 1) {
    const n = CHAPTERS[i + 1];
    pager.innerHTML += `<a class="next" href="#/${n.id}"><span class="label">Next</span>${n.n}. ${n.title}</a>`;
  }
  article.appendChild(pager);
}

async function render(id, wantedLab) {
  const chapter = CHAPTERS.find((c) => c.id === id) || CHAPTERS[0];
  renderSteps(chapter.id);
  article.innerHTML = '<p class="loading">Loading...</p>';
  let text;
  try {
    const res = await fetch(chapter.id + ".md", { cache: "no-cache" });
    if (!res.ok) throw new Error(res.status + " " + res.statusText);
    text = await res.text();
  } catch (err) {
    article.innerHTML = `<p class="error-box">Could not load ${chapter.id}.md (${err.message}). If you opened this file directly from disk, serve the folder instead: <code>python3 -m http.server 8080 --directory docs</code>.</p>`;
    return;
  }

  let html = window.marked.parse(text, { gfm: true });
  html = html.replace(/<pre><code class="language-math">([\s\S]*?)<\/code><\/pre>/g, (_, tex) => renderMath(decodeEntities(tex), true));
  html = html.replace(/\$<code>([\s\S]*?)<\/code>\$/g, (_, tex) => renderMath(decodeEntities(tex), false));
  html = html.replace(/<pre><code class="language-mermaid">([\s\S]*?)<\/code><\/pre>/g, (_, src) => `<div class="mermaid">${src}</div>`);
  article.innerHTML = html;

  // Remove GitHub-only navigation: the breadcrumb line and the trailing "Next" link.
  for (const p of article.querySelectorAll("p")) {
    const first = p.querySelector("a");
    if (first && first.getAttribute("href") === "README.md" && /Docs index/.test(first.textContent)) p.remove();
  }
  for (const p of article.querySelectorAll('p[align="right"]')) p.remove();
  for (const bq of article.querySelectorAll("blockquote")) {
    if (/Interactive version\./.test(bq.textContent)) bq.remove();
  }

  for (const hEl of article.querySelectorAll("h1, h2, h3")) hEl.id = slugify(hEl.textContent);
  for (const a of article.querySelectorAll("a")) rewriteLink(a, chapter.id);

  if (chapter.id === "README") {
    const hero = document.createElement("p");
    hero.setAttribute("align", "center");
    hero.innerHTML = '<img src="assets/svg/hero.svg" alt="Dum-E banner">';
    article.insertBefore(hero, article.firstChild);
  }

  const diagrams = [...article.querySelectorAll(".mermaid")];
  if (diagrams.length && window.mermaid) {
    window.mermaid.initialize({ startOnLoad: false, theme: isDark() ? "dark" : "default", securityLevel: "strict", fontFamily: "Inter, sans-serif" });
    try {
      await window.mermaid.run({ nodes: diagrams });
    } catch (err) {
      console.error(err);
    }
  }

  addPager(chapter.id);
  const visited = new Set(readStore("dume-visited", []));
  visited.add(chapter.id);
  writeStore("dume-visited", [...visited]);
  renderSteps(chapter.id);
  await mountLabs(article, wantedLab);
}

let lastRouted = "";

function route() {
  const hash = location.hash || "#/";
  const m = hash.match(/^#\/([A-Za-z0-9_-]*)(?:\?(.*))?$/);
  const id = m && m[1] ? m[1] : "README";
  const params = new URLSearchParams(m && m[2] ? m[2] : "");
  const key = id + "|" + (params.get("lab") || "");
  if (key !== lastRouted || !article.querySelector(".pager")) window.scrollTo(0, 0);
  lastRouted = key;
  render(id, params.get("lab"));
}

window.addEventListener("hashchange", route);
route();

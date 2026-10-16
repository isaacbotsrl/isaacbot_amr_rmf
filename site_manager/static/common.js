// Copyright 2026 IsaacBot
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Helpers shared by the robots page (index.html) and a robot's page (robot.html).

const $ = (id) => document.getElementById(id);
const esc = (s) => String(s).replace(/[&<>"']/g, (c) => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));

async function api(method, path, body) {
  const res = await fetch(path, {method, headers: {'Content-Type': 'application/json'},
                                 body: body ? JSON.stringify(body) : undefined});
  const data = await res.json().catch(() => ({error: `HTTP ${res.status}`}));
  if (!res.ok) throw new Error(data.error || `HTTP ${res.status}`);
  return data;
}

function ago(seconds) {
  const d = Date.now() / 1000 - seconds;
  if (d < 90) return 'just now';
  if (d < 5400) return `${Math.round(d / 60)} min ago`;
  if (d < 172800) return `${Math.round(d / 3600)} h ago`;
  return new Date(seconds * 1000).toLocaleDateString();
}

// Sets innerHTML only when it changed: the 3 s refresh would otherwise wipe a text selection or
// an open <select>.
const rendered = {};
function setHtml(id, html) {
  if (rendered[id] !== html) $(id).innerHTML = rendered[id] = html;
}

// The header's RMF status, from /api/status.
function renderRmf(status) {
  const rmf = status.rmf || {}, wanted = status.active || rmf.site;
  let cls = 'bad', text = 'RMF is not running';
  if (rmf.state === 'running' && (!status.active || rmf.site === status.active)) {
    cls = 'ok'; text = `RMF runs site ${rmf.site} (since ${ago(rmf.started)})`;
  } else if (wanted) {
    cls = 'warn'; text = `RMF is switching to site ${wanted}…`;
  }
  if (rmf.error) { cls = 'bad'; text += ` — ${rmf.error}`; }
  setHtml('rmf', `<span class="dot ${cls}"></span><span>${esc(text)}</span>`);
}

// Line icons for the Network section (currentColor, so they follow the theme).
const ICONS = {
  server: `<svg class="net-icon" viewBox="0 0 32 32" aria-hidden="true" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round">
    <rect x="5" y="4" width="22" height="10" rx="2"/><rect x="5" y="18" width="22" height="10" rx="2"/>
    <path d="M10 9h.01M10 23h.01M15 9h7M15 23h7"/></svg>`,
  robot: `<svg class="net-icon" viewBox="0 0 32 32" aria-hidden="true" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round">
    <rect x="4" y="12" width="24" height="11" rx="2.5"/><circle cx="10" cy="25" r="2.5"/><circle cx="22" cy="25" r="2.5"/>
    <path d="M13 12V9h6v3M16 9V6M12 17.5h8"/><path d="M11.5 4.5a6 6 0 0 1 9 0"/></svg>`,
};

// Broker session of a robot or another client: online since / last seen.
function sessionState(s) {
  if (!s) return '<span class="status"><span class="dot"></span>not seen since the broker log began</span>';
  return s.online
    ? `<span class="status"><span class="dot ok"></span>connected ${esc(ago(s.since))}</span>`
    : `<span class="status"><span class="dot bad"></span>offline, last seen ${esc(ago(s.last_seen))}</span>`;
}

// A robot's live state from the fleet adapter (null: RMF does not know it yet).
function robotState(r) {
  const live = r.live;
  if (!live) return '<span class="status"><span class="dot"></span>not in RMF yet</span>';
  const online = live.connection === 'ONLINE';
  const cls = !online ? 'bad' : live.in_fleet === false || live.blocked_reason ? 'warn' : 'ok';
  const parts = [online ? (live.operating_mode || 'online') : live.connection.toLowerCase()];
  if (live.battery_soc !== null && live.battery_soc !== undefined) parts.push(`${Math.round(live.battery_soc * 100)} %`);
  if (online && !live.localized) parts.push('not localized');
  const why = live.blocked_reason ? `<div class="muted">${esc(live.blocked_reason)}</div>` : '';
  return `<span class="status"><span class="dot ${cls}"></span>${esc(parts.join(' · '))}</span>${why}`;
}

// The balena variables after adding a robot or rotating its password (shown once).
function showCredentials(target, r, what) {
  const vars = Object.entries(r.balena).map(([k, v]) => `${k}=${v}`).join('\n');
  const once = r.password ? '<b>The password is shown only now</b>; ' : '';
  $(target).innerHTML = `<div class="message ok"><b>${esc(r.robot.name)}</b> ${what}.
    ${once}set these balenaCloud device variables on the isaacbot (each change restarts its VDA 5050 container):
    <div class="cmd"><pre>${esc(vars)}</pre><button data-copy>Copy</button></div></div>`;
}

async function copyText(text) {
  if (window.isSecureContext && navigator.clipboard) return navigator.clipboard.writeText(text);
  // Plain http on the LAN or VPN address has no navigator.clipboard.
  const t = document.createElement('textarea');
  t.value = text; t.style.position = 'fixed'; t.style.opacity = '0';
  document.body.appendChild(t); t.select();
  const ok = document.execCommand('copy');
  t.remove();
  if (!ok) throw new Error('copy refused');
}

async function copyCommand(button) {
  const pre = button.previousElementSibling;
  try {
    await copyText(pre.textContent);
    button.textContent = 'Copied ✓';
  } catch (e) {
    getSelection().selectAllChildren(pre);
    button.textContent = 'Press Ctrl+C';
  }
  setTimeout(() => { button.textContent = 'Copy'; }, 1500);
}

// Click handling: each page passes its own handlers; Copy buttons work everywhere.
function onButtons(handlers) {
  document.addEventListener('click', async (event) => {
    const b = event.target.closest('button');
    if (!b) return;
    try {
      if ('copy' in b.dataset) return await copyCommand(b);
      for (const [key, handler] of Object.entries(handlers)) {
        if (b.id === key || b.dataset[key] !== undefined) return await handler(b, b.dataset[key]);
      }
    } catch (e) { alert(e.message); b.disabled = false; }
  });
}

// The IsaacBot website's header and footer (http://<server>/), on every page: the lockup, the
// page links (current: 'robots' or 'sites'), one call to action (the RMF dashboard), and the
// dark footer.
function siteChrome(current) {
  const link = (href, page, text) =>
    `<a href="${href}" ${page === current ? 'class="current" aria-current="page"' : ''}>${text}</a>`;
  const host = location.hostname;
  const chevron = '<svg viewBox="0 0 16 16" aria-hidden="true"><path d="M6 3l5 5-5 5" fill="none" stroke="currentColor" stroke-width="1.8"></path></svg>';
  $('site-header').outerHTML = `<header class="site-header" id="top"><div class="container">
      <a class="logo" href="/" aria-label="IsaacBot robots">
        <picture><source media="(prefers-color-scheme: dark)" srcset="/isaacbot-lockup-white.svg">
          <img src="/isaacbot-lockup-black-bold.svg" alt="IsaacBot" width="180" height="25"></picture></a>
      <span class="spacer"></span>
      <nav class="site-nav" aria-label="Main">${link('/', 'robots', 'Robots')}${link('/sites', 'sites', 'RMF sites')}</nav>
      <a class="btn" href="http://${host}:3000" target="_blank" rel="noopener">
        <span><span class="wide-only">Open RMF </span>dashboard</span><span class="btn-chevrons">${chevron}${chevron}</span></a>
    </div></header>`;
  $('site-footer').outerHTML = `<footer class="site-footer">
      <div class="container footer-grid">
        <div class="footer-brand"><a href="/" aria-label="IsaacBot robots">
            <img src="/isaacbot-lockup-white.svg" alt="IsaacBot" width="200" height="44" loading="lazy"></a>
          <p>From a falling apple to autonomous logistics.</p></div>
        <div class="footer-col"><h4>Navigate</h4><ul>
          <li><a href="/">Robots</a></li>
          <li><a href="/sites">RMF sites</a></li>
          <li><a href="http://${host}:3000" target="_blank" rel="noopener">RMF dashboard</a></li>
          <li><a href="http://${host}/" target="_blank" rel="noopener">IsaacBot website</a></li></ul></div>
        <div class="footer-col"><h4>Contact</h4><ul>
          <li><a href="mailto:potlogradu@gmail.com">potlogradu@gmail.com</a></li></ul></div>
      </div>
      <div class="container footer-bottom"><span>© ${new Date().getFullYear()} IsaacBot. All rights reserved.</span>
        <a href="#top">Back to top ↑</a></div>
      <img class="footer-wordmark" src="/isaacbot-lockup-white.svg" alt="" aria-hidden="true" loading="lazy">
    </footer>`;
}

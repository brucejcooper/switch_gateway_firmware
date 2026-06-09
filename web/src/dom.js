// Tiny DOM helper. h(tag, props, ...children) -> HTMLElement.
// props: attributes/properties; keys starting with "on" are event listeners.
// children: strings (text), nodes, arrays, or null/false (skipped).

export function h(tag, props, ...children) {
  const el = document.createElement(tag);
  if (props) {
    for (const [k, v] of Object.entries(props)) {
      if (v == null || v === false) continue;
      if (k === 'class') el.className = v;
      else if (k === 'style' && typeof v === 'object') Object.assign(el.style, v);
      else if (k.startsWith('on') && typeof v === 'function') {
        el.addEventListener(k.slice(2).toLowerCase(), v);
      } else if (k === 'html') {
        el.innerHTML = v;
      } else if (k in el && k !== 'list') {
        try { el[k] = v; } catch { el.setAttribute(k, v); }
      } else {
        el.setAttribute(k, v === true ? '' : v);
      }
    }
  }
  append(el, children);
  return el;
}

function append(el, children) {
  for (const c of children) {
    if (c == null || c === false || c === true) continue;
    if (Array.isArray(c)) append(el, c);
    else if (c instanceof Node) el.appendChild(c);
    else el.appendChild(document.createTextNode(String(c)));
  }
}

// Replace the contents of `parent` with `node(s)`.
export function mount(parent, ...nodes) {
  parent.replaceChildren();
  append(parent, nodes);
}

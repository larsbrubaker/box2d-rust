// Debug-draw / view-flag mask shared by sample-shell + wasm sim_set_debug_flags.
// Bit order mirrors demo/wasm/src/interact/draw.rs MENU_* and the C View menu.

export interface ViewFlagDef {
  key: string;
  label: string;
  bit: number;
  default: boolean;
  section: "main" | "contact" | "anchor";
}

export const VIEW_FLAGS: ViewFlagDef[] = [
  { key: "shapes", label: "Shapes", bit: 1 << 0, default: true, section: "main" },
  { key: "chainNormals", label: "Chain Normals", bit: 1 << 1, default: false, section: "main" },
  { key: "joints", label: "Joints", bit: 1 << 2, default: false, section: "main" },
  { key: "jointExtras", label: "Joint Extras", bit: 1 << 3, default: false, section: "main" },
  { key: "bounds", label: "Bounds", bit: 1 << 4, default: false, section: "main" },
  { key: "mass", label: "Mass", bit: 1 << 5, default: false, section: "main" },
  { key: "bodyNames", label: "Body Names", bit: 1 << 6, default: false, section: "main" },
  { key: "graphColors", label: "Graph Colors", bit: 1 << 7, default: false, section: "main" },
  { key: "islands", label: "Islands", bit: 1 << 8, default: false, section: "main" },
  { key: "contacts", label: "Contact Points", bit: 1 << 9, default: false, section: "contact" },
  { key: "contactNormals", label: "Contact Normals", bit: 1 << 10, default: false, section: "contact" },
  { key: "contactFeatures", label: "Contact Features", bit: 1 << 11, default: false, section: "contact" },
  { key: "contactForces", label: "Contact Forces", bit: 1 << 12, default: false, section: "contact" },
  { key: "frictionForces", label: "Friction Forces", bit: 1 << 13, default: false, section: "contact" },
  { key: "anchorA", label: "Anchor A", bit: 1 << 14, default: false, section: "anchor" },
];

export const VIEW_BITS: Record<string, number> = Object.fromEntries(
  VIEW_FLAGS.map((f) => [f.key, f.bit]),
);

export const VIEW_FLAG_DEFAULTS: Record<string, boolean> = Object.fromEntries(
  VIEW_FLAGS.map((f) => [f.key, f.default]),
);

/** Debug-draw panel checkboxes — C View menu order + labels (sample.cpp). */
export const PANEL_FLAG_DEFS: { label: string; viewKey: string }[] = [
  { label: "Shapes", viewKey: "shapes" },
  { label: "Chain Normals", viewKey: "chainNormals" },
  { label: "Joints", viewKey: "joints" },
  { label: "Joint Extras", viewKey: "jointExtras" },
  { label: "Bounds", viewKey: "bounds" },
  { label: "Mass", viewKey: "mass" },
  { label: "Body Names", viewKey: "bodyNames" },
  { label: "Graph Colors", viewKey: "graphColors" },
  { label: "Islands", viewKey: "islands" },
  { label: "Contact Points", viewKey: "contacts" },
  { label: "Contact Normals", viewKey: "contactNormals" },
  { label: "Contact Features", viewKey: "contactFeatures" },
  { label: "Contact Forces", viewKey: "contactForces" },
  { label: "Friction Forces", viewKey: "frictionForces" },
  { label: "Anchor A", viewKey: "anchorA" },
];

export function maskFromFlags(flags: Record<string, boolean>): number {
  let mask = 0;
  for (const f of VIEW_FLAGS) {
    if (flags[f.key]) mask |= f.bit;
  }
  return mask;
}

export function defaultViewFlags(): Record<string, boolean> {
  return { ...VIEW_FLAG_DEFAULTS };
}

// Demo Roadmap — one category per upstream samples app section.

const CATEGORIES: Array<{ name: string; blurb: string; route?: string }> = [
  { name: "Bodies", blurb: "Body types, sleeping, user data", route: "bodies" },
  { name: "Shapes", blurb: "Circles, capsules, polygons, chains", route: "shapes" },
  { name: "Geometry", blurb: "Hulls, rays, and shape queries", route: "geometry" },
  { name: "Collision", blurb: "Manifolds, distance, casting", route: "collision" },
  { name: "Stacking", blurb: "Pyramids, towers, and piles", route: "stacking" },
  { name: "Joints", blurb: "Revolute, prismatic, wheel, weld…", route: "joints" },
  { name: "Continuous", blurb: "Fast bodies without tunneling", route: "continuous" },
  { name: "Events", blurb: "Contacts, sensors, hit events", route: "events" },
  { name: "Character", blurb: "Movers and platforming", route: "character" },
  { name: "World", blurb: "Gravity, explosions, large worlds", route: "world" },
  { name: "Determinism", blurb: "Cross-platform reproducibility", route: "determinism" },
  { name: "Robustness", blurb: "Degenerate input, overlap recovery", route: "robustness" },
  { name: "Benchmark", blurb: "Performance stress scenes", route: "benchmark" },
];

export function init(container: HTMLElement) {
  const cards = CATEGORIES.map((cat) => {
    if (cat.route) {
      return `
        <a href="#/${cat.route}" class="feature-card">
          <h3>${cat.name} <span style="font-size:0.7rem;color:var(--result-stroke);border:1px solid var(--result-stroke);border-radius:10px;padding:1px 8px;vertical-align:middle;">LIVE</span></h3>
          <p>${cat.blurb}</p>
        </a>`;
    }
    return `
      <div class="feature-card" style="opacity:0.6;cursor:default;">
        <h3>${cat.name} <span style="font-size:0.7rem;color:var(--text-muted);border:1px solid var(--border);border-radius:10px;padding:1px 8px;vertical-align:middle;">PLANNED</span></h3>
        <p>${cat.blurb}</p>
      </div>`;
  }).join("");

  container.innerHTML = `
    <div class="home-page">
      <div class="hero">
        <h1>Demo <span>Roadmap</span></h1>
        <p>
          Each category of the upstream <code>samples</code> app becomes an interactive browser
          demo as its module of the engine is ported. The engine already steps the full
          pipeline; remaining cards flip live as their public APIs and sample scenes land.
        </p>
      </div>
      <div class="feature-grid">${cards}</div>
    </div>
  `;
}

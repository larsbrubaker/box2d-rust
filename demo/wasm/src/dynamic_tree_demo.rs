//! Dynamic Tree sample bindings (`sample_collision.cpp` DynamicTree).
//! Browser uses C debug grid defaults (100×100), disclosed as partial.

use wasm_bindgen::prelude::*;

use box2d_rust::dynamic_tree::{DynamicTree, DEFAULT_CATEGORY_BITS};
use box2d_rust::math_functions::{Aabb, Vec2};

struct Proxy {
    position: Vec2,
    width: Vec2,
    box_: Aabb,
    fat_box: Aabb,
    proxy_id: i32,
    ray_stamp: i32,
    query_stamp: i32,
}

#[wasm_bindgen]
pub struct TreeDemo {
    tree: DynamicTree,
    proxies: Vec<Proxy>,
    row_count: i32,
    column_count: i32,
    fill: f32,
    grid: f32,
    ratio: f32,
    move_fraction: f32,
    move_delta: f32,
    time_stamp: i32,
    /// 0 incremental, 1 partial rebuild, 2 full rebuild
    update_type: i32,
    rng: u32,
}

fn aabb_margin() -> Vec2 {
    Vec2 { x: 0.1, y: 0.1 }
}

#[wasm_bindgen]
impl TreeDemo {
    #[wasm_bindgen(constructor)]
    pub fn new() -> TreeDemo {
        let mut d = TreeDemo {
            tree: DynamicTree::new(16),
            proxies: Vec::new(),
            // sample_collision.cpp:489-490 m_isDebug defaults
            row_count: 100,
            column_count: 100,
            fill: 0.25,
            grid: 1.0,
            ratio: 5.0,
            move_fraction: 0.05,
            move_delta: 0.1,
            time_stamp: 0,
            update_type: 0,
            rng: 12345,
        };
        d.build_tree();
        d
    }

    fn next_u(&mut self) -> u32 {
        // C XorShift RandomFloat / RandomFloatRange seed path (utils)
        let mut x = self.rng;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.rng = x;
        x
    }

    fn float_range(&mut self, lo: f32, hi: f32) -> f32 {
        let r = (self.next_u() & 0x7fff) as f32 / 0x7fff as f32;
        (1.0 - r) * lo + r * hi
    }

    fn random_float(&mut self) -> f32 {
        self.float_range(-1.0, 1.0)
    }

    pub fn set_rows(&mut self, rows: i32) {
        self.row_count = rows.clamp(0, 1000);
    }
    pub fn set_columns(&mut self, cols: i32) {
        self.column_count = cols.clamp(0, 1000);
    }
    pub fn set_fill(&mut self, fill: f32) {
        self.fill = fill.clamp(0.0, 1.0);
    }
    pub fn set_grid(&mut self, grid: f32) {
        self.grid = grid.clamp(0.5, 2.0);
    }
    pub fn set_ratio(&mut self, ratio: f32) {
        self.ratio = ratio.clamp(1.0, 10.0);
    }
    pub fn set_move_fraction(&mut self, v: f32) {
        self.move_fraction = v.clamp(0.0, 1.0);
    }
    pub fn set_move_delta(&mut self, v: f32) {
        self.move_delta = v.clamp(0.0, 1.0);
    }
    pub fn set_update_type(&mut self, t: i32) {
        self.update_type = t.clamp(0, 2);
    }

    pub fn row_count(&self) -> i32 {
        self.row_count
    }
    pub fn column_count(&self) -> i32 {
        self.column_count
    }
    pub fn proxy_count(&self) -> i32 {
        self.proxies.len() as i32
    }
    pub fn tree_height(&self) -> i32 {
        self.tree.height()
    }
    pub fn area_ratio(&self) -> f32 {
        self.tree.area_ratio()
    }

    /// Rebuild proxies — sample_collision.cpp:510-571 BuildTree.
    pub fn build_tree(&mut self) {
        self.tree.destroy();
        self.tree = DynamicTree::new(16);
        self.proxies.clear();
        self.rng = 12345;

        let mut y = -4.0;
        let margin = aabb_margin();
        for _i in 0..self.row_count {
            let mut x = -40.0;
            for _j in 0..self.column_count {
                let fill_test = self.float_range(0.0, 1.0);
                if fill_test <= self.fill {
                    let ratio = self.float_range(1.0, self.ratio);
                    let width = self.float_range(0.1, 0.5);
                    let (wx, wy) = if self.random_float() > 0.0 {
                        (ratio * width, width)
                    } else {
                        (width, ratio * width)
                    };
                    let box_ = Aabb {
                        lower_bound: Vec2 { x, y },
                        upper_bound: Vec2 {
                            x: x + wx,
                            y: y + wy,
                        },
                    };
                    let fat = Aabb {
                        lower_bound: Vec2 {
                            x: box_.lower_bound.x - margin.x,
                            y: box_.lower_bound.y - margin.y,
                        },
                        upper_bound: Vec2 {
                            x: box_.upper_bound.x + margin.x,
                            y: box_.upper_bound.y + margin.y,
                        },
                    };
                    let idx = self.proxies.len() as u64;
                    let proxy_id =
                        self.tree
                            .create_proxy(fat, DEFAULT_CATEGORY_BITS, idx);
                    self.proxies.push(Proxy {
                        position: Vec2 { x, y },
                        width: Vec2 { x: wx, y: wy },
                        box_,
                        fat_box: fat,
                        proxy_id,
                        ray_stamp: -1,
                        query_stamp: -1,
                    });
                }
                x += self.grid;
            }
            y += self.grid;
        }
    }

    /// One update step: move some proxies / rebuild. sample_collision.cpp Step.
    pub fn step(&mut self) {
        self.time_stamp = self.time_stamp.wrapping_add(1);
        match self.update_type {
            1 => {
                let _ = self.tree.rebuild(false);
            }
            2 => {
                let _ = self.tree.rebuild(true);
            }
            _ => {
                // Incremental moves
                let n = self.proxies.len();
                for i in 0..n {
                    if self.float_range(0.0, 1.0) > self.move_fraction {
                        continue;
                    }
                    let dx = self.float_range(-self.move_delta, self.move_delta);
                    let dy = self.float_range(-self.move_delta, self.move_delta);
                    let p = &mut self.proxies[i];
                    p.position.x += dx;
                    p.position.y += dy;
                    p.box_.lower_bound.x += dx;
                    p.box_.lower_bound.y += dy;
                    p.box_.upper_bound.x += dx;
                    p.box_.upper_bound.y += dy;
                    let margin = aabb_margin();
                    p.fat_box.lower_bound = Vec2 {
                        x: p.box_.lower_bound.x - margin.x,
                        y: p.box_.lower_bound.y - margin.y,
                    };
                    p.fat_box.upper_bound = Vec2 {
                        x: p.box_.upper_bound.x + margin.x,
                        y: p.box_.upper_bound.y + margin.y,
                    };
                    let id = p.proxy_id;
                    let fat = p.fat_box;
                    self.tree.move_proxy(id, fat);
                }
            }
        }
    }

    /// Leaf AABBs for drawing: `[x0,y0,x1,y1]*N` (tight boxes).
    pub fn leaf_boxes(&self) -> Vec<f32> {
        let mut out = Vec::with_capacity(self.proxies.len() * 4);
        for p in &self.proxies {
            out.push(p.box_.lower_bound.x);
            out.push(p.box_.lower_bound.y);
            out.push(p.box_.upper_bound.x);
            out.push(p.box_.upper_bound.y);
        }
        out
    }

    /// AABB query — stamps query_stamp; returns hit proxy indices.
    pub fn query_aabb(&mut self, x0: f32, y0: f32, x1: f32, y1: f32) -> Vec<i32> {
        let aabb = Aabb {
            lower_bound: Vec2 { x: x0.min(x1), y: y0.min(y1) },
            upper_bound: Vec2 { x: x0.max(x1), y: y0.max(y1) },
        };
        let stamp = self.time_stamp;
        let mut hits = Vec::new();
        let proxies = &mut self.proxies;
        self.tree.query(aabb, u64::MAX, |_id, user_data| {
            let i = user_data as usize;
            if i < proxies.len() {
                proxies[i].query_stamp = stamp;
                hits.push(i as i32);
            }
            true
        });
        hits
    }

    /// Ray cast through tree — returns hit proxy indices.
    pub fn ray_cast(&mut self, ox: f32, oy: f32, ex: f32, ey: f32) -> Vec<i32> {
        let input = box2d_rust::collision::RayCastInput {
            origin: Vec2 { x: ox, y: oy },
            translation: Vec2 {
                x: ex - ox,
                y: ey - oy,
            },
            max_fraction: 1.0,
        };
        let stamp = self.time_stamp;
        let mut hits = Vec::new();
        let proxies = &mut self.proxies;
        self.tree.ray_cast(&input, u64::MAX, |_input, _id, user_data| {
            let i = user_data as usize;
            if i < proxies.len() {
                proxies[i].ray_stamp = stamp;
                hits.push(i as i32);
            }
            1.0 // continue
        });
        hits
    }

    /// Query/ray highlight masks: `[query?, ray?]` per proxy as 0/1 pairs.
    pub fn highlight_flags(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(self.proxies.len() * 2);
        for p in &self.proxies {
            out.push(if p.query_stamp == self.time_stamp {
                1
            } else {
                0
            });
            out.push(if p.ray_stamp == self.time_stamp { 1 } else { 0 });
        }
        out
    }

    pub fn root_bounds(&self) -> Vec<f32> {
        let a = self.tree.root_bounds();
        vec![
            a.lower_bound.x,
            a.lower_bound.y,
            a.upper_bound.x,
            a.upper_bound.y,
        ]
    }
}

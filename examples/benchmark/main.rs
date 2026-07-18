// Port of box2d-cpp-reference/benchmark/main.c: the Box2D benchmark harness.
//
// Single-threaded Rust adaptation. The C harness sweeps 1..maxThreadCount
// workers; the Rust solver is serial, so there is exactly one worker and the
// thread sweep collapses to a single "thread count: 1" pass. The `-t` and `-w`
// options are accepted (for command-line compatibility) but ignored, with a
// note printed. Everything else — benchmark table, step counts, warm-up step,
// timing window, repeat/min logic, `-s` profile dumps and `<name>.csv` output —
// mirrors the C source.
//
// SPDX-FileCopyrightText: 2024 Erin Catto
// SPDX-License-Identifier: MIT

mod human;
mod scenes;
mod utils;

use std::fs::File;
use std::io::Write;

use box2d_rust::timer::{get_milliseconds, get_ticks};
use box2d_rust::types::default_world_def;
use box2d_rust::world::{world_get_counters, world_get_profile, world_step, Profile, World};

/// (b2MinFloat) — kept explicit to match C's `a < b ? a : b` semantics.
fn min_float(a: f32, b: f32) -> f32 {
    if a < b {
        a
    } else {
        b
    }
}

/// (b2ClampInt)
fn clamp_int(a: i32, lo: i32, hi: i32) -> i32 {
    if a < lo {
        lo
    } else if a > hi {
        hi
    } else {
        a
    }
}

type CreateFcn = fn(&mut World);
type StepFcn = fn(&mut World, i32) -> f32;

/// (Benchmark)
struct Benchmark {
    name: &'static str,
    create_fcn: CreateFcn,
    step_fcn: Option<StepFcn>,
    total_step_count: i32,
}

/// (MinProfile) — only the seven columns dumped to the `.dat` files are tracked.
fn min_profile(p1: &mut Profile, p2: &Profile) {
    p1.step = min_float(p1.step, p2.step);
    p1.pairs = min_float(p1.pairs, p2.pairs);
    p1.collide = min_float(p1.collide, p2.collide);
    p1.constraints = min_float(p1.constraints, p2.constraints);
    p1.transforms = min_float(p1.transforms, p2.transforms);
    p1.refit = min_float(p1.refit, p2.refit);
    p1.sleep_islands = min_float(p1.sleep_islands, p2.sleep_islands);
}

/// The seven profile columns start at FLT_MAX so the running minimum works.
/// The other Profile fields are never read by the harness, so they stay zero.
fn max_profile() -> Profile {
    Profile {
        step: f32::MAX,
        pairs: f32::MAX,
        collide: f32::MAX,
        constraints: f32::MAX,
        transforms: f32::MAX,
        refit: f32::MAX,
        sleep_islands: f32::MAX,
        ..Profile::default()
    }
}

fn main() {
    let benchmarks: [Benchmark; 10] = [
        Benchmark {
            name: "compounds",
            create_fcn: scenes::create_compounds,
            step_fcn: None,
            total_step_count: 500,
        },
        Benchmark {
            name: "joint_grid",
            create_fcn: scenes::create_joint_grid,
            step_fcn: None,
            total_step_count: 500,
        },
        Benchmark {
            name: "junkyard",
            create_fcn: scenes::create_junkyard,
            step_fcn: Some(scenes::step_junkyard),
            total_step_count: 800,
        },
        Benchmark {
            name: "large_pyramid",
            create_fcn: scenes::create_large_pyramid,
            step_fcn: None,
            total_step_count: 500,
        },
        Benchmark {
            name: "many_pyramids",
            create_fcn: scenes::create_many_pyramids,
            step_fcn: None,
            total_step_count: 200,
        },
        Benchmark {
            name: "rain",
            create_fcn: scenes::create_rain,
            step_fcn: Some(scenes::step_rain),
            total_step_count: 1000,
        },
        Benchmark {
            name: "smash",
            create_fcn: scenes::create_smash,
            step_fcn: None,
            total_step_count: 300,
        },
        Benchmark {
            name: "spinner",
            create_fcn: scenes::create_spinner,
            step_fcn: Some(scenes::step_spinner),
            total_step_count: 500,
        },
        Benchmark {
            name: "tumbler",
            create_fcn: scenes::create_tumbler,
            step_fcn: None,
            total_step_count: 750,
        },
        Benchmark {
            name: "washer",
            create_fcn: scenes::create_washer,
            step_fcn: None,
            total_step_count: 500,
        },
    ];

    let benchmark_count = benchmarks.len() as i32;

    let mut max_steps = benchmarks[0].total_step_count;
    for b in benchmarks.iter().skip(1) {
        max_steps = max_steps.max(b.total_step_count);
    }

    // Profiles persist across all benchmarks, exactly like the C array that is
    // allocated once before the benchmark loop and never reset.
    let mut profiles: Vec<Profile> = vec![max_profile(); max_steps as usize];
    let mut step_results: Vec<f32> = vec![0.0; max_steps as usize];

    let mut run_count = 4;
    let mut single_benchmark = -1;
    let mut enable_continuous = true;
    let mut record_step_times = false;

    for arg in std::env::args().skip(1) {
        if let Some(value) = arg.strip_prefix("-t=") {
            // Serial port: the worker count is fixed at 1.
            let _ = value.parse::<i32>().unwrap_or(0);
            println!("Note: '-t' ignored; the Rust port runs a single-threaded solver");
        } else if let Some(value) = arg.strip_prefix("-b=") {
            single_benchmark = value.parse::<i32>().unwrap_or(0);
            single_benchmark = clamp_int(single_benchmark, 0, benchmark_count - 1);
        } else if let Some(value) = arg.strip_prefix("-w=") {
            // Serial port: a single worker count is the only option.
            let _ = value.parse::<i32>().unwrap_or(0);
            println!("Note: '-w' ignored; the Rust port runs a single-threaded solver");
        } else if let Some(value) = arg.strip_prefix("-r=") {
            run_count = clamp_int(value.parse::<i32>().unwrap_or(0), 1, 1000);
        } else if arg.starts_with("-nc") {
            enable_continuous = false;
            println!("Continuous disabled");
        } else if arg.starts_with("-s") {
            record_step_times = true;
        } else if arg == "-h" {
            println!(
                "Usage\n\
                 -t=<integer>: the maximum number of threads to use (ignored, serial port)\n\
                 -b=<integer>: run a single benchmark\n\
                 -w=<integer>: run a single worker count (ignored, serial port)\n\
                 -r=<integer>: number of repeats (default is 4)\n\
                 -nc: disable continuous collision\n\
                 -s: record step times"
            );
            std::process::exit(0);
        }
    }

    println!("Starting Box2D benchmarks");
    println!("======================================");

    for benchmark_index in 0..benchmark_count as usize {
        if single_benchmark != -1 && benchmark_index as i32 != single_benchmark {
            continue;
        }

        let benchmark = &benchmarks[benchmark_index];

        // NDEBUG uses the full step count; debug builds cap at 10 like the C.
        let step_count = if cfg!(debug_assertions) {
            10
        } else {
            benchmark.total_step_count
        };

        let mut counters = box2d_rust::types::Counters::default();
        let mut counters_acquired = false;

        println!("benchmark: {}, steps = {}", benchmark.name, step_count);

        // Single thread only in the serial port.
        let mut min_time = 0.0f32;

        println!("thread count: 1");

        for run_index in 0..run_count {
            let world_def = {
                let mut wd = default_world_def();
                wd.enable_continuous = enable_continuous;
                wd.worker_count = 1;
                wd
            };
            let mut world = World::new(&world_def);

            (benchmark.create_fcn)(&mut world);

            let time_step = 1.0 / 60.0;
            let sub_step_count = 4;

            // Initial step can be expensive and skew benchmark.
            if let Some(step_fcn) = benchmark.step_fcn {
                step_results[0] = step_fcn(&mut world, 0);
            }

            debug_assert!(step_count <= max_steps);

            world_step(&mut world, time_step, sub_step_count);

            let profile = world_get_profile(&world);
            min_profile(&mut profiles[0], &profile);

            let ticks = get_ticks();

            for step_index in 1..step_count {
                if let Some(step_fcn) = benchmark.step_fcn {
                    step_results[step_index as usize] = step_fcn(&mut world, step_index);
                }

                world_step(&mut world, time_step, sub_step_count);
                let profile = world_get_profile(&world);
                min_profile(&mut profiles[step_index as usize], &profile);
            }

            let ms = get_milliseconds(ticks);
            println!("run {} : {} (ms)", run_index, ms);

            if run_index == 0 {
                min_time = ms;
            } else {
                min_time = min_float(min_time, ms);
            }

            if !counters_acquired {
                counters = world_get_counters(&world);
                counters_acquired = true;
            }

            // b2DestroyWorld: dropping the world frees it.
            drop(world);
        }

        if record_step_times {
            let file_name = format!("{}_t1.dat", benchmark.name);
            if let Ok(mut file) = File::create(&file_name) {
                for step_index in 0..step_count as usize {
                    let p = profiles[step_index];
                    let _ = writeln!(
                        file,
                        "{} {} {} {} {} {} {}",
                        p.step,
                        p.pairs,
                        p.collide,
                        p.constraints,
                        p.transforms,
                        p.refit,
                        p.sleep_islands
                    );
                }
            }
        }

        println!(
            "body {} / shape {} / contact {} / joint {} / stack {}",
            counters.body_count,
            counters.shape_count,
            counters.contact_count,
            counters.joint_count,
            counters.stack_used
        );
        print!("color counts:");
        for c in counters.color_counts.iter() {
            print!(" {}", c);
        }
        println!("\n");

        let file_name = format!("{}.csv", benchmark.name);
        if let Ok(mut file) = File::create(&file_name) {
            let _ = writeln!(file, "threads,ms");
            let _ = writeln!(file, "1,{}", min_time);
        }
    }

    println!("======================================");
    println!("All Box2D benchmarks complete!");
}

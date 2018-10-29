use std::env;

fn main() {
    let target = env::var("TARGET").unwrap();

    if target.starts_with("thumbv7m-") {
        println!("cargo:rustc-cfg=armv7m");
    }

    println!("cargo:rerun-if-changed=build.rs");
}

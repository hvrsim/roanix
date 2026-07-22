use std::process::Command;

fn main() {
    let arch = std::env::var("CARGO_CFG_TARGET_ARCH").unwrap();

    // Pass linker file into cargo.
    println!("cargo:rustc-link-arg=-Tlinker-{arch}.ld");
    println!("cargo:rerun-if-changed=linker-{arch}.ld");

    let git_hash = git_output(&["rev-parse", "--short=12", "HEAD"])
        .unwrap_or_else(|| "unknown".to_string());
    println!("cargo:rustc-env=ROANIX_GIT_HASH={git_hash}");
    println!("cargo:rerun-if-changed=../.git/HEAD");
    if let Some(reference) = git_output(&["symbolic-ref", "-q", "HEAD"]) {
        println!("cargo:rerun-if-changed=../.git/{reference}");
    }
}

fn git_output(arguments: &[&str]) -> Option<String> {
    let output = Command::new("git")
        .arg("-C")
        .arg("..")
        .args(arguments)
        .output()
        .ok()?;
    output
        .status
        .success()
        .then(|| String::from_utf8_lossy(&output.stdout).trim().to_string())
}

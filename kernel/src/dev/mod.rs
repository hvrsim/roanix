//! Kernel-owned device services.
//!
//! Hardware access remains in loadable drivers. Namespace policy, terminal
//! semantics, pseudo-devices, and other software-only facilities live here.

pub mod block;
pub mod console;
pub mod drm;
pub mod node;
pub mod pty;
pub mod special;
pub mod tty;

/// Initializes kernel-owned device services.
pub fn init() {
    block::init();
    console::init();
}

/// Publishes software device nodes after devfs has been mounted.
pub fn publish() {
    node::create_directory(None, node::root().expect("dev: devfs root"), "input", 0o755)
        .expect("dev: failed to publish input directory");
    drm::publish().expect("dev: failed to publish boot DRM device");
    special::init().expect("dev: failed to publish special devices");
    pty::init().expect("dev: failed to initialize pseudo terminals");
}

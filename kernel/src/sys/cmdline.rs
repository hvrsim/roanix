//! Kernel-owned boot command line, copied before bootloader memory is reclaimed.

use limine::request::ExecutableCmdlineRequest;

use super::sync::Once;

const MAX_COMMAND_LINE: usize = 4096;

#[used]
#[unsafe(link_section = ".requests")]
static REQUEST: ExecutableCmdlineRequest = ExecutableCmdlineRequest::new();

struct CommandLine {
    bytes: [u8; MAX_COMMAND_LINE],
    length: usize,
}

static COMMAND_LINE: Once<CommandLine> = Once::new();

/// Captures the bootloader response without allocating or retaining its pointers.
/// Called on the boot CPU before memory initialization and reclamation.
pub(crate) fn init() {
    COMMAND_LINE.call_once(|| {
        let source = REQUEST
            .get_response()
            .map(|response| response.cmdline().to_bytes())
            .unwrap_or_default();
        assert!(
            source.len() <= MAX_COMMAND_LINE,
            "kernel command line exceeds 4096 bytes"
        );
        let mut command_line = CommandLine {
            bytes: [0; MAX_COMMAND_LINE],
            length: source.len(),
        };
        command_line.bytes[..source.len()].copy_from_slice(source);
        command_line
    });
}

/// Returns the exact command-line bytes supplied at boot, without a terminator.
pub(crate) fn get() -> &'static [u8] {
    let command_line = COMMAND_LINE
        .get()
        .expect("boot command line not initialized");
    &command_line.bytes[..command_line.length]
}

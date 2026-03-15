//!
//! # Shared Framebuffer Request
//!
//! Centralized Limine framebuffer request used by all framebuffer consumers in
//! the kernel. Limine expects each request type to be declared once.
//!

use limine::request::FramebufferRequest;

#[used]
#[doc(hidden)]
#[link_section = ".requests"]
pub(crate) static FRAMEBUFFER_REQUEST: FramebufferRequest = FramebufferRequest::new();

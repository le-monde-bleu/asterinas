//! The util of jinux
#![no_std]
#![forbid(unsafe_code)]

extern crate alloc;

pub mod dup;
pub mod frame_ptr;
pub mod safe_ptr;
pub mod slot_vec;
pub mod union_read_ptr;
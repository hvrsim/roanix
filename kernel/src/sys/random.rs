//! Kernel pseudorandom byte generator.

use crate::{
    arch,
    sys::{
        clock,
        sync::{Mutex, Once},
    },
};

const BLOCK_SIZE: usize = 64;
const CHACHA_CONSTANTS: [u32; 4] = [0x6170_7865, 0x3320_646e, 0x7962_2d32, 0x6b20_6574];

static GENERATOR: Once<Mutex<Generator>> = Once::new();

struct Generator {
    key: [u32; 8],
    counter: u64,
    nonce: [u32; 2],
    block: [u8; BLOCK_SIZE],
    used: usize,
}

/// Initializes the kernel random generator.
pub(crate) fn init() {
    let _ = generator();
}

/// Fills `output` with pseudorandom bytes.
pub(crate) fn fill_bytes(output: &mut [u8]) {
    if output.is_empty() {
        return;
    }

    let mut generator = generator().lock();
    generator.stir_runtime();
    generator.fill(output);
}

/// Mixes caller-provided data into the generator state.
pub(crate) fn mix_bytes(input: &[u8]) {
    if input.is_empty() {
        return;
    }

    let mut generator = generator().lock();
    for (index, chunk) in input.chunks(8).enumerate() {
        let mut word = [0u8; 8];
        word[..chunk.len()].copy_from_slice(chunk);
        generator.mix_word(u64::from_le_bytes(word) ^ index as u64);
    }
}

fn generator() -> &'static Mutex<Generator> {
    GENERATOR.call_once(|| Mutex::new(Generator::seeded()))
}

impl Generator {
    fn seeded() -> Self {
        let marker = 0u8;
        let mut seed = clock::monotonic_ns()
            ^ (&marker as *const u8 as usize as u64).rotate_left(17)
            ^ (&GENERATOR as *const Once<Mutex<Generator>> as usize as u64).rotate_left(41)
            ^ (arch::thiscpu_opt().map_or(0, |cpu| cpu.id) as u64).rotate_left(29);

        let mut key = [0u32; 8];
        for word in &mut key {
            if let Some(entropy) = arch::entropy_word() {
                seed ^= entropy;
            }
            seed = splitmix64(seed ^ clock::monotonic_ns());
            *word = seed as u32;
        }

        let nonce_seed = splitmix64(seed ^ clock::monotonic_ns());
        let counter = splitmix64(nonce_seed);
        Self {
            key,
            counter,
            nonce: [nonce_seed as u32, (nonce_seed >> 32) as u32],
            block: [0; BLOCK_SIZE],
            used: BLOCK_SIZE,
        }
    }

    fn stir_runtime(&mut self) {
        let marker = 0u8;
        let timing = clock::monotonic_ns()
            ^ (&marker as *const u8 as usize as u64).rotate_left(23)
            ^ (arch::thiscpu_opt().map_or(0, |cpu| cpu.id) as u64).rotate_left(47);
        self.mix_word(timing);
        if let Some(entropy) = arch::entropy_word() {
            self.mix_word(entropy);
        }
    }

    fn mix_word(&mut self, word: u64) {
        let mut mixer = word ^ self.counter ^ u64::from(self.nonce[0]).rotate_left(32);
        for (index, key) in self.key.iter_mut().enumerate() {
            mixer = splitmix64(mixer ^ u64::from(*key) ^ index as u64);
            *key ^= (mixer ^ (mixer >> 32)) as u32;
        }
        self.nonce[0] ^= mixer as u32;
        self.nonce[1] ^= (mixer >> 32) as u32;
        self.counter = self.counter.wrapping_add(splitmix64(mixer));
        self.used = BLOCK_SIZE;
    }

    fn fill(&mut self, output: &mut [u8]) {
        let mut written = 0;
        while written < output.len() {
            if self.used == BLOCK_SIZE {
                self.refill();
            }
            let count = (BLOCK_SIZE - self.used).min(output.len() - written);
            output[written..written + count]
                .copy_from_slice(&self.block[self.used..self.used + count]);
            self.used += count;
            written += count;
        }
    }

    fn refill(&mut self) {
        let mut state = [
            CHACHA_CONSTANTS[0],
            CHACHA_CONSTANTS[1],
            CHACHA_CONSTANTS[2],
            CHACHA_CONSTANTS[3],
            self.key[0],
            self.key[1],
            self.key[2],
            self.key[3],
            self.key[4],
            self.key[5],
            self.key[6],
            self.key[7],
            self.counter as u32,
            (self.counter >> 32) as u32,
            self.nonce[0],
            self.nonce[1],
        ];
        let original = state;

        for _ in 0..10 {
            quarter_round(&mut state, 0, 4, 8, 12);
            quarter_round(&mut state, 1, 5, 9, 13);
            quarter_round(&mut state, 2, 6, 10, 14);
            quarter_round(&mut state, 3, 7, 11, 15);
            quarter_round(&mut state, 0, 5, 10, 15);
            quarter_round(&mut state, 1, 6, 11, 12);
            quarter_round(&mut state, 2, 7, 8, 13);
            quarter_round(&mut state, 3, 4, 9, 14);
        }

        for (index, word) in state.iter_mut().enumerate() {
            *word = word.wrapping_add(original[index]);
            self.block[index * 4..index * 4 + 4].copy_from_slice(&word.to_le_bytes());
        }
        self.counter = self.counter.wrapping_add(1);
        self.used = 0;
    }
}

fn quarter_round(state: &mut [u32; 16], a: usize, b: usize, c: usize, d: usize) {
    let (mut a_value, mut b_value, mut c_value, mut d_value) =
        (state[a], state[b], state[c], state[d]);

    a_value = a_value.wrapping_add(b_value);
    d_value ^= a_value;
    d_value = d_value.rotate_left(16);
    c_value = c_value.wrapping_add(d_value);
    b_value ^= c_value;
    b_value = b_value.rotate_left(12);
    a_value = a_value.wrapping_add(b_value);
    d_value ^= a_value;
    d_value = d_value.rotate_left(8);
    c_value = c_value.wrapping_add(d_value);
    b_value ^= c_value;
    b_value = b_value.rotate_left(7);

    state[a] = a_value;
    state[b] = b_value;
    state[c] = c_value;
    state[d] = d_value;
}

fn splitmix64(mut value: u64) -> u64 {
    value = value.wrapping_add(0x9e37_79b9_7f4a_7c15);
    value = (value ^ (value >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    value = (value ^ (value >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    value ^ (value >> 31)
}

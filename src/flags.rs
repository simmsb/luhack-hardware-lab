const KEY: &[u8] = b"deez";

pub const fn encrypt_inner<const LEN: usize>(s: &[u8]) -> [u8; LEN] {
    let mut out = [0u8; LEN];
    let mut i = 0;
    while i < s.len() {
        out[i] = s[i] ^ KEY[i % KEY.len()];
        i += 1;
    }

    out
}

#[inline(never)]
pub fn decrypt<const LEN: usize>(s: &[u8; LEN]) -> [u8; LEN] {
    let mut out = [0u8; LEN];
    let mut i = 0;

    while i < s.len() {
        out[i] = s[i] ^ KEY[i % KEY.len()];
        i += 1;
    }

    out
}

#[macro_export]
macro_rules! encrypt {
    ($s:expr) => {
        {
            static Y: [u8; { $s.len() }] = $crate::flags::encrypt_inner($s);
            $crate::flags::decrypt(&Y)
        }
    };
}

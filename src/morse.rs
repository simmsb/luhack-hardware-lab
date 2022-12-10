#[derive(Clone, Copy, defmt::Format)]
pub enum Morse {
    Dit,
    Dah,
    Brk,
}

impl Morse {
    pub fn wait_ms(&self) -> u64 {
        match self {
            Self::Dit => 500,
            Self::Dah => 1500,
            Self::Brk => 700,
        }
    }

    pub fn lit(&self) -> bool {
        match self {
            Dit | Dah => true,
            Brk => false,
        }
    }

    pub const INTERVAL_MS: u64 = 500;
}

use Morse::*;

static MAP: phf::Map<char, &'static [Morse]> = phf::phf_map! {
    'a' => &[Dit, Dah],
    'b' => &[Dah, Dit, Dit, Dit],
    'c' => &[Dah, Dit, Dah, Dit],
    'd' => &[Dah, Dit, Dit],
    'e' => &[Dit],
    'f' => &[Dit, Dit, Dah, Dit],
    'g' => &[Dah, Dah, Dit],
    'h' => &[Dit, Dit, Dit, Dit],
    'i' => &[Dit, Dit],
    'j' => &[Dit, Dah, Dah, Dah],
    'k' => &[Dah, Dit, Dah],
    'l' => &[Dit, Dah, Dit, Dit],
    'm' => &[Dah, Dah],
    'n' => &[Dah, Dit],
    'o' => &[Dah, Dah, Dah],
    'p' => &[Dit, Dah, Dah, Dit],
    'q' => &[Dah, Dah, Dit, Dah],
    'r' => &[Dit, Dah, Dit],
    's' => &[Dit, Dit, Dit],
    't' => &[Dah],
    'u' => &[Dit, Dit, Dah],
    'v' => &[Dit, Dit, Dit, Dah],
    'w' => &[Dit, Dah, Dah],
    'x' => &[Dah, Dit, Dit, Dah],
    'y' => &[Dah, Dit, Dah, Dah],
    'z' => &[Dah, Dah, Dit, Dit],
    '1' => &[Dit, Dah, Dah, Dah, Dah],
    '2' => &[Dit, Dit, Dah, Dah, Dah],
    '3' => &[Dit, Dit, Dit, Dah, Dah],
    '4' => &[Dit, Dit, Dit, Dit, Dah],
    '5' => &[Dit, Dit, Dit, Dit, Dit],
    '6' => &[Dah, Dit, Dit, Dit, Dit],
    '7' => &[Dah, Dah, Dit, Dit, Dit],
    '8' => &[Dah, Dah, Dah, Dit, Dit],
    '9' => &[Dah, Dah, Dah, Dah, Dit],
    '0' => &[Dah, Dah, Dah, Dah, Dah],
    '_' => &[Dit, Dit, Dah, Dah, Dit, Dah],
    '(' => &[Dah, Dit, Dah, Dah, Dit],
    ')' => &[Dah, Dit, Dah ,Dah, Dit, Dah],
};

pub fn convert(s: &str) -> impl Iterator<Item = Morse> + '_ {
    s.chars()
        .map(|c| c.to_ascii_lowercase())
        .filter_map(|c| MAP.get(&c).copied())
        .flat_map(|x| x.iter().copied().chain(core::iter::once(Brk)))
}

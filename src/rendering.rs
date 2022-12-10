use core::cell::UnsafeCell;

use cichlid::prelude::ScalingInt;
use derive_more::{Add, Mul};
use embedded_graphics::draw_target::DrawTargetExt;
use embedded_graphics::image::Image;
use embedded_graphics::pixelcolor::{Rgb565, Rgb888};
use embedded_graphics::prelude::{DrawTarget, OriginDimensions, Point, Size};
use embedded_graphics::primitives::Rectangle;
use embedded_graphics::transform::Transform;
use embedded_graphics::Drawable;
use euc::{fixed_32::Fixed32, CullMode, Nearest, Pipeline, Sampler, Target, Texture, TriangleList};
use fixed::traits::{FromFixed, ToFixed};
use fixed::FixedU8;
use hal::gpio;
use mipidsi::ModelOptions;
use mipidsi::models::{Model, ST7789};
use rp_pico::hal;
use vek::{Mat4, Rgba, Vec2, Vec3, Vec4};

macro_rules! fixed {
    ($e:expr) => {{
        const X: Fixed32 = Fixed32::unwrapped_from_str(stringify!($e));
        X
    }};
}

pub struct FixedST7789(pub ST7789);

impl Model for FixedST7789 {
    type ColorFormat = <ST7789 as Model>::ColorFormat;

    fn init<RST, DELAY, DI>(
        &mut self,
        di: &mut DI,
        delay: &mut DELAY,
        madctl: u8,
        rst: &mut Option<RST>,
    ) -> Result<u8, mipidsi::error::InitError<RST::Error>>
    where
        RST: eh_0_2::digital::v2::OutputPin,
        DELAY: cortex_m::prelude::_embedded_hal_blocking_delay_DelayUs<u32>,
        DI: display_interface::WriteOnlyDataCommand {
        self.0.init(di, delay, madctl, rst)
    }

    fn write_pixels<DI, I>(
        &mut self,
        di: &mut DI,
        colors: I,
    ) -> Result<(), display_interface::DisplayError>
    where
        DI: display_interface::WriteOnlyDataCommand,
        I: IntoIterator<Item = Self::ColorFormat>,
    {
        self.0.write_pixels(di, colors)
    }

    fn default_options() -> mipidsi::ModelOptions {
        ModelOptions::with_all((172, 320), (240, 320), offset_handler)
    }
}

fn offset_handler(options: &ModelOptions) -> (u16, u16) {
    (0, 34)
}

pub struct MyDisplay(
    pub  mipidsi::Display<
        display_interface_spi::SPIInterface<
            hal::Spi<hal::spi::Enabled, rp_pico::pac::SPI1, 8>,
            hal::gpio::Pin<gpio::bank0::Gpio15, gpio::Output<gpio::PushPull>>,
            gpio::Pin<gpio::bank0::Gpio13, gpio::Output<gpio::PushPull>>,
        >,
        FixedST7789,
        gpio::Pin<gpio::bank0::Gpio14, gpio::Output<gpio::PushPull>>,
    >,
);

unsafe impl Send for MyDisplay {}
unsafe impl Sync for MyDisplay {}

pub const WIDTH: usize = 80;
pub const HEIGHT: usize = 80;
pub const FULL_WIDTH: usize = 320;
pub const FULL_HEIGHT: usize = 172;

pub struct DepthBuffer<const X: usize, const Y: usize>([[FixedU8<7>; X]; Y]);

impl<const X: usize, const Y: usize> DepthBuffer<X, Y> {
    pub const fn new() -> Self {
        DepthBuffer([[FixedU8::<7>::unwrapped_from_str("0"); X]; Y])
    }
}

pub type MyDepthBuffer = DepthBuffer<WIDTH, HEIGHT>;

struct LolWrapper<'a, T>(UnsafeCell<&'a mut T>);

unsafe impl<'a, T> Send for LolWrapper<'a, T> {}
unsafe impl<'a, T> Sync for LolWrapper<'a, T> {}

impl<'a, T> LolWrapper<'a, T> {
    pub fn new(inner: &'a mut T) -> Self {
        Self(UnsafeCell::new(inner))
    }

    pub fn undo(self) -> &'a mut T {
        unsafe { self.0.get().as_mut().unwrap() }
    }

    #[allow(clippy::mut_from_ref)]
    pub unsafe fn i_promise(&self) -> &mut T {
        self.0.get().as_mut().unwrap()
    }
}

impl<'a, const X: usize, const Y: usize> Texture<2> for LolWrapper<'a, DepthBuffer<X, Y>> {
    type Index = usize;

    type Texel = Fixed32;

    fn size(&self) -> [Self::Index; 2] {
        [X, Y]
    }

    fn read(&self, index: [Self::Index; 2]) -> Self::Texel {
        let [x, y] = index;
        unsafe { self.i_promise() }.0[y][x].into()
    }
}

impl<'a, const X: usize, const Y: usize> Target for LolWrapper<'a, DepthBuffer<X, Y>> {
    unsafe fn read_exclusive_unchecked(&self, index: [Self::Index; 2]) -> Self::Texel {
        self.read(index)
    }

    unsafe fn write_exclusive_unchecked(&self, index: [usize; 2], texel: Self::Texel) {
        let [x, y] = index;
        self.i_promise().0[y][x] = texel.unsigned_abs().to_num();
    }
}

pub struct FrameBuffer<const X: usize, const Y: usize>([[Rgba<u8>; X]; Y]);

impl<const X: usize, const Y: usize> FrameBuffer<X, Y> {
    pub const fn new() -> Self {
        FrameBuffer([[Rgba::new(0, 0, 0, 255); X]; Y])
    }
}

pub type MyFrameBuffer = FrameBuffer<WIDTH, HEIGHT>;

impl<'a, const X: usize, const Y: usize> IntoIterator for &'a FrameBuffer<X, Y> {
    type Item = Rgb565;

    type IntoIter = impl Iterator<Item = Rgb565>;

    fn into_iter(self) -> Self::IntoIter {
        self.0.into_iter().flat_map(|row| {
            row.map(|p| {
                let [r, g, b, a] = p.into_array();
                let r = r.scale(a);
                let g = g.scale(a);
                let b = b.scale(a);

                Rgb888::new(r, g, b).into()
            })
        })
    }
}

impl<'a, const X: usize, const Y: usize> Texture<2> for LolWrapper<'a, FrameBuffer<X, Y>> {
    type Index = usize;

    type Texel = Rgba<u8>;

    fn size(&self) -> [Self::Index; 2] {
        [X, Y]
    }

    fn read(&self, index: [Self::Index; 2]) -> Self::Texel {
        let [x, y] = index;
        unsafe { self.i_promise() }.0[y][x]
    }
}

impl<'a, const X: usize, const Y: usize> Target for LolWrapper<'a, FrameBuffer<X, Y>> {
    unsafe fn read_exclusive_unchecked(&self, index: [Self::Index; 2]) -> Self::Texel {
        self.read(index)
    }

    unsafe fn write_exclusive_unchecked(&self, index: [usize; 2], texel: Self::Texel) {
        let [x, y] = index;
        self.i_promise().0[y][x] = texel;
    }
}

static LUHACK_LOGO: &[u8] = include_bytes!("logo.rgb");
static LUHACK_TEXT: &[u8] = include_bytes!("logo_text.bmp");

#[derive(Clone, Copy)]
struct ImageRawSampler<const X: usize, const Y: usize>(&'static [u8]);

static LUHACK_LOGO_SAMPLER: ImageRawSampler<WIDTH, HEIGHT> = ImageRawSampler(LUHACK_LOGO);

impl<const X: usize, const Y: usize> Texture<2> for ImageRawSampler<X, Y> {
    type Index = usize;

    type Texel = Rgba<Fixed32>;

    fn size(&self) -> [Self::Index; 2] {
        [X, Y]
    }

    fn read(&self, [x, y]: [Self::Index; 2]) -> Self::Texel {
        let data_width = 3;
        let row_width = X * data_width;

        let idx = data_width * y + x * row_width;
        let slice = idx..idx + 3;

        let [r, g, b] = self.0[slice] else { unreachable!() };
        // defmt::info!("reading from image at {}: {}", [x, y], c.map(f32::from_fixed).into_array());

        Rgba::new(
            r.to_fixed::<Fixed32>() / 255,
            g.to_fixed::<Fixed32>() / 255,
            b.to_fixed::<Fixed32>() / 255,
            1i8.to_fixed(),
        )
    }
}

#[derive(Add, Mul, Clone)]
struct VertexData {
    light: Fixed32,
    uv: Vec2<Fixed32>,
}

struct Cube<'a> {
    locked: bool,
    vp: Mat4<Fixed32>,
    m: Mat4<Fixed32>,
    // v: Mat4<Fixed32>,
    // p: Mat4<Fixed32>,
    light_pos: Vec3<Fixed32>,
    positions: &'a [Vec3<Fixed32>],
    normals: &'a [Vec3<Fixed32>],
    uvs: &'a [Vec2<Fixed32>],
    sampler: &'a Nearest<ImageRawSampler<WIDTH, HEIGHT>>,
}

fn normalize(m: Vec3<Fixed32>) -> Vec3<Fixed32> {
    m / cordic::sqrt(m.magnitude_squared())
}

impl<'a> Pipeline for Cube<'a> {
    type Vertex = usize;
    type VertexData = VertexData;
    type Primitives = TriangleList;
    type Fragment = Rgba<Fixed32>;
    type Pixel = Rgba<u8>;

    #[inline(always)]
    fn depth_mode(&self) -> euc::DepthMode {
        euc::DepthMode::LESS_WRITE
    }

    #[inline(always)]
    fn vertex_shader(&self, v_index: &Self::Vertex) -> ([Fixed32; 4], Self::VertexData) {
        let wpos = self.m * Vec4::from_point(-self.positions[*v_index]);
        let wnorm = self.m * Vec4::from_direction(self.normals[*v_index]);
        let wnorm = normalize(wnorm.xyz());

        let light_dir = normalize(self.light_pos - wpos.xyz());
        let cam_dir = normalize(-wpos.xyz());

        let ambient = fixed!(0.2);
        let diffuse = wnorm.dot(light_dir).max(fixed!(0.0)) * fixed!(0.9);
        let specular = (-light_dir).reflected(wnorm).dot(cam_dir).max(fixed!(0.0));
        let specular = specular * specular;
        let specular = specular * specular;
        let specular = specular * fixed!(5.0);
        let light = ambient + diffuse + specular;

        let v = (self.vp * wpos).into_array();
        let uv = self.uvs[*v_index];

        let d = VertexData { light, uv };

        (v, d)
    }

    #[inline(always)]
    fn fragment_shader(&self, VertexData { light, uv }: Self::VertexData) -> Self::Fragment {
        let colour = self.sampler.sample(uv.into_array());

        // let specular = (-light_dir).reflected(wnorm).dot(-cam_dir).max(fixed!(0.0));

        // let light = ambient + diffuse // + specular
        //     ;

        colour * light

        // defmt::info!("uv: {}", v_uv.map(|x| f32::from_fixed(x)).into_array());
    }

    #[inline(always)]
    fn blend_shader(&self, _: Self::Pixel, new: Self::Fragment) -> Self::Pixel {
        let [r, g, b, a]: [u8; 4] = new
            .map(|x| u8::saturating_from_fixed(x.saturating_mul_int(255)))
            .into_array();
        if self.locked {
        // let [r, g, b, a] = (new * 255.0).as_().into_array();
            Rgba::new(r, g, b, a)
        } else {
            Rgba::new(g, r, b, a)
        }
    }
}

#[rustfmt::skip]
static VERTICES: &[Vec3<Fixed32>] = &[
    // z = 1
    Vec3::new(fixed!(-1.0), fixed!(-1.0), fixed!(1.0)),
    Vec3::new(fixed!(-1.0), fixed!(1.0), fixed!(1.0)),
    Vec3::new(fixed!(1.0), fixed!(1.0), fixed!(1.0)),
    Vec3::new(fixed!(1.0), fixed!(-1.0), fixed!(1.0)),
    // z == -1
    Vec3::new(fixed!(-1.0), fixed!(-1.0), fixed!(-1.0)),
    Vec3::new(fixed!(-1.0), fixed!(1.0), fixed!(-1.0)),
    Vec3::new(fixed!(1.0), fixed!(1.0), fixed!(-1.0)),
    Vec3::new(fixed!(1.0), fixed!(-1.0), fixed!(-1.0)),
    // y = 1
    Vec3::new(fixed!(-1.0), fixed!(1.0), fixed!(1.0)),
    Vec3::new(fixed!(-1.0), fixed!(1.0), fixed!(-1.0)),
    Vec3::new(fixed!(1.0), fixed!(1.0), fixed!(-1.0)),
    Vec3::new(fixed!(1.0), fixed!(1.0), fixed!(1.0)),
    // y = -1
    Vec3::new(fixed!(-1.0), fixed!(-1.0), fixed!(1.0)),
    Vec3::new(fixed!(-1.0), fixed!(-1.0), fixed!(-1.0)),
    Vec3::new(fixed!(1.0), fixed!(-1.0), fixed!(-1.0)),
    Vec3::new(fixed!(1.0), fixed!(-1.0), fixed!(1.0)),
    // x = 1
    Vec3::new(fixed!(1.0), fixed!(-1.0), fixed!(1.0)),
    Vec3::new(fixed!(1.0), fixed!(-1.0), fixed!(-1.0)),
    Vec3::new(fixed!(1.0), fixed!(1.0), fixed!(-1.0)),
    Vec3::new(fixed!(1.0), fixed!(1.0), fixed!(1.0)),
    // x = -1
    Vec3::new(fixed!(-1.0), fixed!(-1.0), fixed!(1.0)),
    Vec3::new(fixed!(-1.0), fixed!(-1.0), fixed!(-1.0)),
    Vec3::new(fixed!(-1.0), fixed!(1.0), fixed!(-1.0)),
    Vec3::new(fixed!(-1.0), fixed!(1.0), fixed!(1.0)),
];

#[rustfmt::skip]
static NORMALS: &[Vec3<Fixed32>] = &[
    // z = 1
    Vec3::new(fixed!(0.0), fixed!(0.0), fixed!(1.0)),
    Vec3::new(fixed!(0.0), fixed!(0.0), fixed!(1.0)),
    Vec3::new(fixed!(0.0), fixed!(0.0), fixed!(1.0)),
    Vec3::new(fixed!(0.0), fixed!(0.0), fixed!(1.0)),
    // z = -1
    Vec3::new(fixed!(0.0), fixed!(0.0), fixed!(-1.0)),
    Vec3::new(fixed!(0.0), fixed!(0.0), fixed!(-1.0)),
    Vec3::new(fixed!(0.0), fixed!(0.0), fixed!(-1.0)),
    Vec3::new(fixed!(0.0), fixed!(0.0), fixed!(-1.0)),
    // y = 1
    Vec3::new(fixed!(0.0), fixed!(1.0), fixed!(0.0)),
    Vec3::new(fixed!(0.0), fixed!(1.0), fixed!(0.0)),
    Vec3::new(fixed!(0.0), fixed!(1.0), fixed!(0.0)),
    Vec3::new(fixed!(0.0), fixed!(1.0), fixed!(0.0)),
    // y = -1
    Vec3::new(fixed!(0.0), fixed!(-1.0), fixed!(0.0)),
    Vec3::new(fixed!(0.0), fixed!(-1.0), fixed!(0.0)),
    Vec3::new(fixed!(0.0), fixed!(-1.0), fixed!(0.0)),
    Vec3::new(fixed!(0.0), fixed!(-1.0), fixed!(0.0)),
    // x = 1
    Vec3::new(fixed!(1.0), fixed!(0.0), fixed!(0.0)),
    Vec3::new(fixed!(1.0), fixed!(0.0), fixed!(0.0)),
    Vec3::new(fixed!(1.0), fixed!(0.0), fixed!(0.0)),
    Vec3::new(fixed!(1.0), fixed!(0.0), fixed!(0.0)),
    // x = -1
    Vec3::new(fixed!(-1.0), fixed!(0.0), fixed!(0.0)),
    Vec3::new(fixed!(-1.0), fixed!(0.0), fixed!(0.0)),
    Vec3::new(fixed!(-1.0), fixed!(0.0), fixed!(0.0)),
    Vec3::new(fixed!(-1.0), fixed!(0.0), fixed!(0.0)),
];

#[rustfmt::skip]
static UVS: &[Vec2<Fixed32>] = &[
    // z = 1
    Vec2::new(fixed!(0.0), fixed!(1.0)),
    Vec2::new(fixed!(0.0), fixed!(0.0)),
    Vec2::new(fixed!(1.0), fixed!(0.0)),
    Vec2::new(fixed!(1.0), fixed!(1.0)),
    // z = -1
    Vec2::new(fixed!(0.0), fixed!(0.0)),
    Vec2::new(fixed!(0.0), fixed!(1.0)),
    Vec2::new(fixed!(1.0), fixed!(1.0)),
    Vec2::new(fixed!(1.0), fixed!(0.0)),
    // y = 1
    Vec2::new(fixed!(0.0), fixed!(0.0)),
    Vec2::new(fixed!(0.0), fixed!(1.0)),
    Vec2::new(fixed!(1.0), fixed!(1.0)),
    Vec2::new(fixed!(1.0), fixed!(0.0)),
    // y = -1
    Vec2::new(fixed!(0.0), fixed!(0.0)),
    Vec2::new(fixed!(0.0), fixed!(1.0)),
    Vec2::new(fixed!(1.0), fixed!(1.0)),
    Vec2::new(fixed!(1.0), fixed!(0.0)),
    // x = 1
    Vec2::new(fixed!(1.0), fixed!(1.0)),
    Vec2::new(fixed!(1.0), fixed!(0.0)),
    Vec2::new(fixed!(0.0), fixed!(0.0)),
    Vec2::new(fixed!(0.0), fixed!(1.0)),
    // x = -1
    Vec2::new(fixed!(0.0), fixed!(1.0)),
    Vec2::new(fixed!(0.0), fixed!(0.0)),
    Vec2::new(fixed!(1.0), fixed!(0.0)),
    Vec2::new(fixed!(1.0), fixed!(1.0)),
];

static INDICES: &[usize] = &[
    0, 3, 1, 1, 3, 2, 4, 5, 7, 5, 6, 7, 8, 11, 9, 9, 11, 10, 12, 13, 15, 13, 14, 15, 16, 17, 19,
    17, 18, 19, 20, 23, 21, 21, 23, 22,
];

pub fn draw_nth_frame(
    display: &mut MyDisplay,
    frame: &mut MyFrameBuffer,
    depth: &mut MyDepthBuffer,
    n: usize,
    locked: bool,
) {
    let n = n % 1534;

    let p = perspective_fov_lh_zo(
        fixed!(1.1),
        WIDTH.to_fixed(),
        HEIGHT.to_fixed(),
        fixed!(0.01),
        fixed!(100.0),
    );
    let v = translation_3d(Vec3::new(fixed!(0.0), fixed!(0.0), fixed!(4.0)));
    let m = rotation_x(cordic::sin(n.to_fixed::<Fixed32>() * fixed!(0.0082)) * fixed!(8.0))
        * rotation_y(cordic::cos(n.to_fixed::<Fixed32>() * fixed!(0.0041)) * fixed!(8.0));
    let vp = p * v;

    let light_pos = Vec3::new(fixed!(-5.0), fixed!(-5.0), fixed!(10.0));

    let mut frame = LolWrapper::new(frame);
    let mut depth = LolWrapper::new(depth);

    frame.clear(Rgba::new(0, 0, 0, 255));
    depth.clear(fixed!(1.0));

    let cube = Cube {
        locked,
        m,
        vp,
        // v,
        // p,
        light_pos,
        positions: VERTICES,
        normals: NORMALS,
        uvs: UVS,
        sampler: &Nearest::new(LUHACK_LOGO_SAMPLER),
    };

    cube.render(INDICES, CullMode::Back, &mut frame, &mut depth);

    let rect = Rectangle::new(
        Point::new(4, (FULL_HEIGHT - HEIGHT) as i32 / 2 - 8),
        Size::new(WIDTH as u32, HEIGHT as u32),
    );

    display
        .0
        .fill_contiguous(&rect, frame.undo().into_iter())
        .ok()
        .unwrap();

    if n == 0 {
        let text_image = tinybmp::Bmp::<Rgb565>::from_slice(LUHACK_TEXT).unwrap();

        let Size { width, height } = text_image.size();

        defmt::info!("image size: {} {}", width, height);

        let logo_end = rect.bottom_right().unwrap().x;
        let text_x = logo_end + 4;
        let text_y = (FULL_HEIGHT - HEIGHT) as i32 / 2;

        let image = Image::new(&text_image, Point::new(0, 0)).translate(Point::new(text_x, text_y));
        image.draw(&mut display.0).ok().unwrap();
    }
}

#[allow(unused)]
fn scaling_3d(v: Vec3<Fixed32>) -> Mat4<Fixed32> {
    let Vec3 { x, y, z } = v;

    let one = Fixed32::ONE;
    let zero = Fixed32::ZERO;
    Mat4::new(
        x, zero, zero, zero, zero, y, zero, zero, zero, zero, z, zero, zero, zero, zero, one,
    )
}

fn rotation_y(angle_radians: Fixed32) -> Mat4<Fixed32> {
    let c = cordic::cos(angle_radians);
    let s = cordic::sin(angle_radians);
    let one = Fixed32::ONE;
    let zero = Fixed32::ZERO;
    Mat4::new(
        c, zero, s, zero, zero, one, zero, zero, -s, zero, c, zero, zero, zero, zero, one,
    )
}

fn rotation_x(angle_radians: Fixed32) -> Mat4<Fixed32> {
    let c = cordic::cos(angle_radians);
    let s = cordic::sin(angle_radians);
    let one = Fixed32::ONE;
    let zero = Fixed32::ZERO;
    Mat4::new(
        one, zero, zero, zero, zero, c, -s, zero, zero, s, c, zero, zero, zero, zero, one,
    )
}

fn translation_3d(by: Vec3<Fixed32>) -> Mat4<Fixed32> {
    Mat4::translation_3d(by)
}

fn perspective_fov_lh_zo(
    fov_y_radians: Fixed32,
    width: Fixed32,
    height: Fixed32,
    near: Fixed32,
    far: Fixed32,
) -> Mat4<Fixed32> {
    let mut m = {
        let two = fixed!(2);
        let rad = fov_y_radians;
        let h = cordic::cos(rad / two) / cordic::sin(rad / two);
        let w = h * height / width;
        let m00 = w;
        let m11 = h;
        let m22 = far / (near - far);
        let m23 = -(far * near) / (far - near);
        let m32 = fixed!(-1);
        let zero = fixed!(0);
        Mat4::new(
            m00, zero, zero, zero, zero, m11, zero, zero, zero, zero, m22, m23, zero, zero, m32,
            zero,
        )
    };
    m[(2, 2)] = -m[(2, 2)];
    m[(3, 2)] = -m[(3, 2)];
    m
}

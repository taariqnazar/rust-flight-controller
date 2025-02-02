use libm::{atan2f, sqrtf};

const RAD_TO_DEG: f32 = 180.0 / core::f32::consts::PI;

pub trait Filter {
    fn update(
        &mut self,
        ax: f32,
        ay: f32,
        az: f32,
        gx: f32,
        gy: f32,
        gz: f32,
        dt: f32,
    ) -> (f32, f32);
}

pub struct ComplementaryFilter {
    alpha: f32,
    angle_x: f32,
    angle_y: f32,
}

impl ComplementaryFilter {
    pub fn new(alpha: f32) -> Self {
        Self {
            alpha,
            angle_x: 0.0,
            angle_y: 0.0,
        }
    }
}

impl Filter for ComplementaryFilter {
    fn update(
        &mut self,
        ax: f32,
        ay: f32,
        az: f32,
        gx: f32,
        gy: f32,
        _: f32,
        dt: f32,
    ) -> (f32, f32) {
        let a_angle_x = atan2f(ay, sqrtf(ax * ax + az * az)) * RAD_TO_DEG;
        let a_angle_y = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

        let g_angle_x = self.angle_x + dt * gx;
        let g_angle_y = self.angle_y + dt * gy;

        let angle_x = (1.0 -self.alpha) * g_angle_x + self.alpha * a_angle_x;
        let angle_y = (1.0 -self.alpha) * g_angle_y + self.alpha * a_angle_y;

        self.angle_x = 0.1 * angle_x + (1.0 - 0.1)*self.angle_x;
        self.angle_y = 0.1 * angle_y + (1.0 - 0.1)*self.angle_y;

        (self.angle_x, self.angle_y)
    }
}

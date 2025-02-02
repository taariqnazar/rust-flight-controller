pub trait Filter<T> {
    fn update(&mut self, data: T) -> T;
}

struct ComplementaryFilter {
    alpha: f32,
}
impl Filter for ComplementaryFilter {}

pub struct KalmanFilter {
    x: [f32; 4],
    p: [[f32; 4]; 4],
    a: [[f32; 4]; 4],
    h: [[f32; 4]; 2],
    q: [[f32; 4]; 4],
    r: [[f32; 4]; 4],
    dt: f32,
}
impl Filter for KalmanFilter {
    fn new(dt: f32) -> Self {
        Self {
            x: [0.0, 0.0, 0.0, 0.0],
            p: [
                [0.1, 0.0, 0.0, 0.0],
                [0.0, 0.1, 0.0, 0.0],
                [0.0, 0.0, 0.1, 0.0],
                [0.0, 0.0, 0.0, 0.1],
            ],
            a: [
                [1.0, 0.0, dt, 0.0],
                [0.0, 1.0, 0.0, dt],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            h: [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]],
            q: [
                [0.001, 0.0, 0.0, 0.0],
                [0.0, 0.001, 0.0, 0.0],
                [0.0, 0.0, 0.001, 0.0],
                [0.0, 0.0, 0.0, 0.001],
            ],
            r: [
                [0.03, 0.0, 0.0, 0.0],
                [0.0, 0.03, 0.0, 0.0],
                [0.0, 0.0, 0.01, 0.0],
                [0.0, 0.0, 0.0, 0.01],
            ],
            dt,
        }
    }

    fn predict(&mut self) {
        let next_x= [
            self.x[0] + self.x[2]*dt,
            self.x[1] + self.x[4]*dt,
            self.x[2],
            self.x[3]];

        let self.x = next_x;
        
        for i in 0..4 {
            for j in 0..4 {
                self.p[i][j] += self.q[i][j];
            }
        }
    }

    fn update(&mut self, z:[f32;4]){
        
    }

}

// ベジェ曲線補間モジュール
// P0=(0,0), P1=(ax,ay), P2=(bx,by), P3=(1,1) の3次ベジェ補間（0-1正規化座標系）

/// 3次ベジェ補間曲線（P0=(0,0), P1=(ax,ay), P2=(bx,by), P3=(1,1)）
#[derive(Debug, Clone, Copy)]
pub struct BezierCurve {
    ax: f64,
    ay: f64,
    bx: f64,
    by: f64,
}

impl BezierCurve {
    pub fn new(ax: f64, ay: f64, bx: f64, by: f64) -> Self {
        BezierCurve { ax, ay, bx, by }
    }

    /// 線形補間かどうか（P1とP2が対角線上: ax==ay かつ bx==by）
    pub fn is_linear(&self) -> bool {
        self.ax == self.ay && self.bx == self.by
    }

    /// x ∈ [0,1] に対応する y 値を返す（2分探索でtを求めてYを計算）
    pub fn evaluate(&self, x: f64) -> f64 {
        let t = self.find_t(x);
        self.bezier_y(t)
    }

    fn find_t(&self, target_x: f64) -> f64 {
        let mut lo = 0.0_f64;
        let mut hi = 1.0_f64;
        let mut t = target_x;
        for _ in 0..20 {
            let diff = self.bezier_x(t) - target_x;
            if diff.abs() < 1e-6 { break; }
            if diff > 0.0 { hi = t; } else { lo = t; }
            t = (lo + hi) / 2.0;
        }
        t
    }

    fn bezier_x(&self, t: f64) -> f64 {
        let mt = 1.0 - t;
        3.0 * mt * mt * t * self.ax + 3.0 * mt * t * t * self.bx + t * t * t
    }

    fn bezier_y(&self, t: f64) -> f64 {
        let mt = 1.0 - t;
        3.0 * mt * mt * t * self.ay + 3.0 * mt * t * t * self.by + t * t * t
    }
}

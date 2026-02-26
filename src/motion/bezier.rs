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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_is_linear_true() {
        let b = BezierCurve::new(0.5, 0.5, 0.7, 0.7);
        assert!(b.is_linear());
    }

    #[test]
    fn test_is_linear_false_ax_ne_ay() {
        let b = BezierCurve::new(0.3, 0.5, 0.7, 0.7);
        assert!(!b.is_linear());
    }

    #[test]
    fn test_is_linear_false_bx_ne_by() {
        let b = BezierCurve::new(0.5, 0.5, 0.6, 0.8);
        assert!(!b.is_linear());
    }

    #[test]
    fn test_evaluate_endpoints() {
        let b = BezierCurve::new(0.3, 0.5, 0.7, 0.8);
        assert!((b.evaluate(0.0) - 0.0).abs() < 1e-5, "evaluate(0) != 0");
        assert!((b.evaluate(1.0) - 1.0).abs() < 1e-5, "evaluate(1) != 1");
    }

    #[test]
    fn test_evaluate_linear_identity() {
        // 線形ベジェ (ax==ay, bx==by) では evaluate(t) ≈ t
        let b = BezierCurve::new(0.25, 0.25, 0.75, 0.75);
        for i in 0..=10 {
            let t = i as f64 / 10.0;
            assert!(
                (b.evaluate(t) - t).abs() < 1e-4,
                "t={}: evaluate={:.6} expected={:.6}", t, b.evaluate(t), t
            );
        }
    }

    #[test]
    fn test_evaluate_nonlinear_easing() {
        // ax=0.1, ay=0.9 → fast-start (curve above diagonal at midpoint)
        let b = BezierCurve::new(0.1, 0.9, 0.1, 0.9);
        assert!(!b.is_linear());
        let mid = b.evaluate(0.5);
        assert!(mid > 0.5, "Expected fast-start mid > 0.5, got {}", mid);
    }

    #[test]
    fn test_evaluate_mmd_default_interp() {
        // MMD デフォルト補間: 20/127 と 107/127 → is_linear (20/127 == 20/127, 107/127 == 107/127)
        let ax = 20.0 / 127.0;
        let bx = 107.0 / 127.0;
        let b = BezierCurve::new(ax, ax, bx, bx);
        assert!(b.is_linear());
    }

    #[test]
    fn test_evaluate_monotone() {
        // 任意の曲線で evaluate は単調増加
        let b = BezierCurve::new(0.2, 0.8, 0.8, 0.2);
        let mut prev = -1.0_f64;
        for i in 0..=20 {
            let t = i as f64 / 20.0;
            let y = b.evaluate(t);
            assert!(y >= prev - 1e-6, "not monotone: t={}, y={}, prev={}", t, y, prev);
            prev = y;
        }
    }
}

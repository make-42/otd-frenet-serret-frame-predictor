#nullable enable
using System;
using System.Collections.Generic;
using System.Numerics;
using OpenTabletDriver.Plugin;
using OpenTabletDriver.Plugin.Attributes;
using OpenTabletDriver.Plugin.Output;
using OpenTabletDriver.Plugin.Tablet;
using System.Diagnostics;
using OpenTabletDriver.Plugin.Logging;

namespace FrenetSerretFramePredictor
{
    [PluginName("OnTake's Frenet-Serret Frame Predictor")]
    public class FrenetSerretFramePredictor : AsyncPositionedPipelineElement<IDeviceReport>
    {
        public FrenetSerretFramePredictor() : base()
        {
        }

        [SliderProperty("Capture Window (ms)", 1f, 500f, 20f),
        DefaultPropertyValue(20f),
        ToolTip("How many milliseconds of past input to use when fitting the prediction curve")]
        public float CaptureWindowMs { get; set; } = 20f;

        [SliderProperty("Look-ahead (ms)", 0f, 50f, 8f),
        DefaultPropertyValue(8f),
        ToolTip("How many milliseconds into the future to predict")]
        public float LookAheadMs { get; set; } = 8f;

        [SliderProperty("1st Derivative Strength (Velocity)", 0f, 2f, 1f),
        DefaultPropertyValue(1f),
        ToolTip("How strongly to apply velocity-based prediction")]
        public float D1Strength { get; set; } = 1f;

        [SliderProperty("2nd Derivative Strength (Acceleration)", 0f, 2f, 1f),
        DefaultPropertyValue(1f),
        ToolTip("How strongly to apply acceleration-based prediction")]
        public float D2Strength { get; set; } = 1f;

        [SliderProperty("3rd Derivative Strength (Jerk)", 0f, 2f, 0f),
        DefaultPropertyValue(1f),
        ToolTip("How strongly to apply jerk-based prediction")]
        public float D3Strength { get; set; } = 0f;

        [SliderProperty("4th Derivative Strength (Snap)", 0f, 2f, 0f),
        DefaultPropertyValue(0f),
        ToolTip("How strongly to apply snap-based prediction")]
        public float D4Strength { get; set; } = 0f;

        [SliderProperty("5th Derivative Strength (Crackle)", 0f, 2f, 0f),
        DefaultPropertyValue(0f),
        ToolTip("How strongly to apply crackle-based prediction")]
        public float D5Strength { get; set; } = 0f;

        [SliderProperty("Rigidity", 0f, 1f, 1f),
        DefaultPropertyValue(1f),
        ToolTip("How strongly to apply correction on each update tick")]
        public float Rigidity { get; set; } = 1f;

        [SliderProperty("Tablet Frequency (Hz)", 0f, 1000f, 133f),
        DefaultPropertyValue(133f),
        ToolTip("How often the tablet sends reports")]
        public float TabFreq { get; set; } = 133f;

        private readonly record struct Sample(float X, float Y, double Tick);
        private readonly List<Sample> _history = new();

        private ITabletReport? _tablet;
        private Vector2 _lastRawPosition;
        private Vector2 _lastPredictedPosition;
        private Vector2 _lastForcedPosition;

        private float[] dxArr = Array.Empty<float>();
        private float[] dyArr = Array.Empty<float>();
        private float[] ndx = Array.Empty<float>();
        private float[] ndy = Array.Empty<float>();
        private float[] vel = Array.Empty<float>();
        private float[] nvel = Array.Empty<float>();
        private float[] normalaccel = Array.Empty<float>();
        private float[] nnormalaccel = Array.Empty<float>();

        private int splinePointsPerSample = 1;
        private float[] splineBaseFunction = Array.Empty<float>();
        private int bsplineOrder = 3;

        private long _lastTick;
        private long tabletReports;
        private readonly Stopwatch _clock = Stopwatch.StartNew();

        private float[] velorders = new float[5];
        private float[] normalaccelorders = new float[5];

        private int integrationsteps = 30;


       protected override void ConsumeState()
        {
            if (State is ITabletReport tablet)
            {
                if (_tablet == null) {
                    _lastForcedPosition = tablet.Position;
                }

                _tablet = tablet;

                long now = _clock.ElapsedTicks;
                _lastTick = now;

                tabletReports += 1;

                _lastRawPosition = tablet.Position;

                _history.Add(new Sample(
                    tablet.Position.X,
                    tablet.Position.Y,
                    tabletReports
                ));

                long cutoff =
                    tabletReports -
                    (long)(CaptureWindowMs * TabFreq / 1000.0);
                _history.RemoveAll(s => s.Tick < cutoff);
            }
        }

        protected override void UpdateState()
        {
            if (_tablet == null)
                return;

            long now = _clock.ElapsedTicks;

            double elapsedMs =
                (now - _lastTick) * 1000.0 / Stopwatch.Frequency;

            var (dx, dy) = PredictDelta(elapsedMs);

            //Log.Write("FrenetSerretFramePredictor", $"dx={dx} dy={dy}", LogLevel.Info);

            _lastPredictedPosition = new Vector2(
                _lastRawPosition.X + dx,
                _lastRawPosition.Y + dy
            );

            _lastForcedPosition = new Vector2(_lastForcedPosition.X+Rigidity*(_lastPredictedPosition.X-_lastForcedPosition.X),_lastForcedPosition.Y+Rigidity*(_lastPredictedPosition.Y-_lastForcedPosition.Y));

            _tablet.Position = _lastForcedPosition;

            OnEmit();
        }

        private void InitSpline()
        {
            int numPoints = splinePointsPerSample * bsplineOrder;
            splineBaseFunction = new float[numPoints];
            for (int i = 0; i < numPoints; i++)
            {
                splineBaseFunction[i] = (float)BaseSpline(0, bsplineOrder, (double)i / splinePointsPerSample);
            }
        }

        private double BaseSpline(int i, int p, double t)
        {
            if (p == 0)
            {
                return (i <= t && t < i + 1) ? 1.0 : 0.0;
            }
            return (t - i) / p * BaseSpline(i, p - 1, t) + (i + p + 1 - t) / p * BaseSpline(i + 1, p - 1, t);
        }


        private (float dx, float dy) PredictDelta(double elapsedMs)
{
    int z = _history.Count;
    if (z < 2) return (0f, 0f);

    if (splineBaseFunction.Length == 0)
        InitSpline();

    float invdt = TabFreq * splinePointsPerSample;
    int n = z * splinePointsPerSample;

    // Evaluate spline into position arrays
    var sx = new float[n];
    var sy = new float[n];
    for (int i = 0; i < n; i++)
    {
        sx[i] = _history[0].X;
        sy[i] = _history[0].Y;
        for (int j = 0; j < z; j++)
        {
            int lo = j * splinePointsPerSample;
            int hi = (j + bsplineOrder) * splinePointsPerSample;
            if (i >= lo && i < hi)
            {
                int idx = i - lo;
                if (idx < splineBaseFunction.Length)
                {
                    sx[i] += (_history[j].X - _history[0].X) * splineBaseFunction[idx];
                    sy[i] += (_history[j].Y - _history[0].Y) * splineBaseFunction[idx];
                }
            }
        }
    }

    // First derivative of spline (velocity vectors)
    int vLen = n - 1;
    var vx = new float[vLen];
    var vy = new float[vLen];
    for (int i = 0; i < vLen; i++)
    {
        vx[i] = (sx[i + 1] - sx[i]) * invdt;
        vy[i] = (sy[i + 1] - sy[i]) * invdt;
    }

    // Weighted average velocity at the trailing end (recent = higher weight)
    float wvx = 0f, wvy = 0f, wvSum = 0f;
    for (int i = 0; i < vLen; i++)
    {
        float w = i + 1f;
        wvx += vx[i] * w;
        wvy += vy[i] * w;
        wvSum += w;
    }
    wvx /= wvSum;
    wvy /= wvSum;

    // Derivative tower: up to 5 orders of velocity vector derivatives
    // velDx[k], velDy[k] = weighted average of (k+1)-th derivative of position
    var velDx = new float[5];
    var velDy = new float[5];

    float[] strengths = { D1Strength, D2Strength, D3Strength, D4Strength, D5Strength };

    var curX = vx;
    var curY = vy;
    int curLen = vLen;

    for (int order = 0; order < 5; order++)
    {
        int nextLen = curLen - 1;
        if (nextLen <= 0) break;

        var nextX = new float[nextLen];
        var nextY = new float[nextLen];
        for (int i = 0; i < nextLen; i++)
        {
            nextX[i] = (curX[i + 1] - curX[i]) * invdt;
            nextY[i] = (curY[i + 1] - curY[i]) * invdt;
        }

        float wx = 0f, wy = 0f, ws = 0f;
        for (int i = 0; i < nextLen; i++)
        {
            float w = (i + 1f)*(i + 1f);
            wx += nextX[i] * w;
            wy += nextY[i] * w;
            ws += w;
        }
        if (ws > 0f)
        {
            velDx[order] = strengths[order] * wx / ws;
            velDy[order] = strengths[order] * wy / ws;
        }

        curX = nextX;
        curY = nextY;
        curLen = nextLen;
    }

    // Normal acceleration (signed curvature × speed²) for turning
    int naLen = n - 2;
    var na = new float[naLen];
    for (int i = 0; i < naLen; i++)
    {
        float ax = vx[i], ay = vy[i];
        float bx = vx[i + 1], by = vy[i + 1];
        float na_ = ax * by - ay * bx; // cross product = signed curvature × speed²
        float speed = MathF.Sqrt(ax * ax + ay * ay);
        na[i] = speed > 1e-6f ? na_ / speed : 0f; // normalise to angular velocity
    }

    float wna = 0f, wnaSum = 0f;
    for (int i = 0; i < naLen; i++)
    {
        float w = (i + 1f)*(i + 1f);
        wna += na[i] * w;
        wnaSum += w;
    }
    float baseNA = wnaSum > 0f ? wna / wnaSum : 0f;

    // Derivative tower for normal acceleration
    var naDx = new float[5];
    var curNa = na;
    int curNaLen = naLen;

    for (int order = 0; order < 5; order++)
    {
        int nextLen = curNaLen - 1;
        if (nextLen <= 0) break;

        var nextNa = new float[nextLen];
        for (int i = 0; i < nextLen; i++)
            nextNa[i] = (curNa[i + 1] - curNa[i]) * invdt;

        float wn = 0f, ws = 0f;
        for (int i = 0; i < nextLen; i++)
        {
            float w = (i + 1f)*(i + 1f);
            wn += nextNa[i] * w;
            ws += w;
        }
        if (ws > 0f)
            naDx[order] = strengths[order] * wn / ws;

        curNa = nextNa;
        curNaLen = nextLen;
    }

    float naScale = Math.Clamp((float)(LookAheadMs / 50.0), 0f, 1f);
    baseNA *= naScale;
    for (int i = 0; i < 5; i++) naDx[i] *= naScale;

    // Integrate predicted path
    double totalT = (LookAheadMs + elapsedMs) / 1000.0;
    float dt = (float)totalT / integrationsteps;
    float dx = 0f, dy = 0f;
    float cvx = wvx, cvy = wvy;

    for (int step = 0; step < integrationsteps; step++)
    {
        float stepT = (step + 0.5f) * dt;

        // Taylor correction to velocity vector from higher derivatives
        float dvx = 0f, dvy = 0f;
        float factorial = 1f;
        for (int order = 0; order < 5; order++)
        {
            factorial *= (order + 1);
            float tPow = MathF.Pow(stepT, order + 1);
            dvx += velDx[order] * tPow / factorial;
            dvy += velDy[order] * tPow / factorial;
        }

        float rvx = cvx + dvx;
        float rvy = cvy + dvy;

        // Taylor expansion of normal acceleration at this step
        float normalAcc = baseNA;
        factorial = 1f;
        for (int order = 0; order < 5; order++)
        {
            factorial *= (order + 1);
            float tPow = MathF.Pow(stepT, order + 1);
            normalAcc += naDx[order] * tPow / factorial;
        }

        // Rotate velocity by angular velocity
        float spd = MathF.Sqrt(rvx * rvx + rvy * rvy);
        float dTheta = spd > 0.0001f ? normalAcc * dt / spd : 0f;
        dTheta = Math.Clamp(dTheta, -0.15f, 0.15f);
        float cosT = MathF.Cos(dTheta);
        float sinT = MathF.Sin(dTheta);

        float rx = rvx * cosT - rvy * sinT;
        float ry = rvx * sinT + rvy * cosT;

        dx += rx * dt;
        dy += ry * dt;

        // Advance base velocity by first derivative
        cvx += velDx[0] * dt;
        cvy += velDy[0] * dt;
    }

    return (dx, dy);
}

        public override PipelinePosition Position => PipelinePosition.PostTransform;
    }
}
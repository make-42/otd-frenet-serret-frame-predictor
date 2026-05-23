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

        [SliderProperty("Spline Points Per Sample", 1f, 8f, 1f),
        DefaultPropertyValue(1),
        ToolTip("Number of spline interpolation points between each tablet sample. (Tune carefully as lots of spline points could cause the time step to be too small and therefore amplify any floating point arithmetic errors)")]
        public int SplinePointsPerSample
        {
            get => splinePointsPerSample;
            set
            {
                splinePointsPerSample = value;
                _splineBaseFunction = Array.Empty<float>(); // force reinit
                lock (_historyLock) { _historyDirty = true; }
            }
        }

        [SliderProperty("Integration Steps", 1f, 100f, 10f),
        DefaultPropertyValue(10),
        ToolTip("Number of steps used to integrate the predicted path. Higher = more accurate prediction, more CPU. (Same comment as spline points per sample)")]
        public int IntegrationSteps
        {
            get => integrationsteps;
            set => integrationsteps = value;
        }

        private readonly record struct Sample(float X, float Y, double Tick);

        private readonly List<Sample> _history = new();
        private bool _historyDirty = true;
        private readonly List<Sample> _historySnapshot = new();
        private readonly object _historyLock = new();

        private ITabletReport? _tablet;
        private Vector2 _lastRawPosition;
        private Vector2 _lastPredictedPosition;
        private Vector2 _lastForcedPosition;

        private int splinePointsPerSample = 1;
        private float[] _splineBaseFunction = Array.Empty<float>();
        private int bsplineOrder = 3;

        private long _lastTick;
        private long _tabletReports;
        private readonly Stopwatch _clock = Stopwatch.StartNew();

        // Cached computations
        private float[] _sx = Array.Empty<float>();
        private float[] _sy = Array.Empty<float>();
        private float[] _vx = Array.Empty<float>();
        private float[] _vy = Array.Empty<float>();
        private float[] _towerA_X = Array.Empty<float>();
        private float[] _towerA_Y = Array.Empty<float>();
        private float[] _towerB_X = Array.Empty<float>();
        private float[] _towerB_Y = Array.Empty<float>();
        private float[] _nextNa_A = Array.Empty<float>();
        private float[] _nextNa_B = Array.Empty<float>();
        private float[] _velDx = new float[5];
        private float[] _velDy = new float[5];
        private float[] _na = Array.Empty<float>();
        private float[] _naDx = new float[5];
        private float _baseNA;
        private float _wvx, _wvy;
        private float[] _strengths = new float[5];

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

                _tabletReports += 1;

                long cutoff =
                    _tabletReports -
                    (long)(CaptureWindowMs * TabFreq / 1000.0);

                lock (_historyLock)
                {
                _lastRawPosition = tablet.Position;
                
                _history.Add(new Sample(
                    tablet.Position.X,
                    tablet.Position.Y,
                    _tabletReports
                ));
                _history.RemoveAll(s => s.Tick < cutoff);
                _historyDirty = true;
                }
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

            Vector2 rawPos;
            lock (_historyLock)
            {
                rawPos = _lastRawPosition;
            }

            _lastPredictedPosition = new Vector2(
                rawPos.X + dx,
                rawPos.Y + dy
            );

            _lastForcedPosition = new Vector2(_lastForcedPosition.X+Rigidity*(_lastPredictedPosition.X-_lastForcedPosition.X),_lastForcedPosition.Y+Rigidity*(_lastPredictedPosition.Y-_lastForcedPosition.Y));
            
            _tablet.Position = _lastForcedPosition;

            OnEmit();
        }

        private void InitSpline()
        {
            int numPoints = splinePointsPerSample * bsplineOrder;
            _splineBaseFunction = new float[numPoints];
            for (int i = 0; i < numPoints; i++)
            {
                _splineBaseFunction[i] = (float)BaseSpline(0, bsplineOrder, (double)i / splinePointsPerSample);
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

        private static void EnsureSize(ref float[] arr, int size)
        {
            if (arr.Length < size)
                arr = new float[size];
        }

        private void RebuildCache()
        {
            _strengths[0] = D1Strength;
            _strengths[1] = D2Strength;
            _strengths[2] = D3Strength;
            _strengths[3] = D4Strength;
            _strengths[4] = D5Strength;

            int z = _historySnapshot.Count;
            if (z < 2) return;

            if (_splineBaseFunction.Length == 0)
                InitSpline();

            float invdt = TabFreq * splinePointsPerSample;
            int n = z * splinePointsPerSample;

            // Evaluate spline into position arrays
            EnsureSize(ref _sx, n);
            EnsureSize(ref _sy, n);
            for (int i = 0; i < n; i++)
            {
                _sx[i] = _historySnapshot[0].X;
                _sy[i] = _historySnapshot[0].Y;
                for (int j = 0; j < z; j++)
                {
                    int lo = j * splinePointsPerSample;
                    int hi = (j + bsplineOrder) * splinePointsPerSample;
                    if (i >= lo && i < hi)
                    {
                        int idx = i - lo;
                        if (idx < _splineBaseFunction.Length)
                        {
                            _sx[i] += (_historySnapshot[j].X - _historySnapshot[0].X) * _splineBaseFunction[idx];
                            _sy[i] += (_historySnapshot[j].Y - _historySnapshot[0].Y) * _splineBaseFunction[idx];
                        }
                    }
                }
            }

            // First derivative of spline (velocity vectors)
            int vLen = n - 1;
            EnsureSize(ref _vx, vLen);
            EnsureSize(ref _vy, vLen);
            for (int i = 0; i < vLen; i++)
            {
                _vx[i] = (_sx[i + 1] - _sx[i]) * invdt;
                _vy[i] = (_sy[i + 1] - _sy[i]) * invdt;
            }

            // Weighted average velocity at the trailing end (recent = higher weight)
            _wvx = 0f;
            _wvy = 0f;
            float wvSum = 0f;

            for (int i = 0; i < vLen; i++)
            {
                float w = i + 1f;
                _wvx += _vx[i] * w;
                _wvy += _vy[i] * w;
                wvSum += w;
            }
            _wvx /= wvSum;
            _wvy /= wvSum;

            // Derivative tower: up to 5 orders of velocity vector derivatives
            // _velDx[k], _velDy[k] = weighted average of (k+1)-th derivative of position
            var curX = _vx;
            var curY = _vy;
            int curLen = vLen;

            for (int order = 0; order < 5; order++)
            {
                int nextLen = curLen - 1;
                if (nextLen <= 0) break;

                ref float[] nextX = ref (order % 2 == 0 ? ref _towerA_X : ref _towerB_X);
                ref float[] nextY = ref (order % 2 == 0 ? ref _towerA_Y : ref _towerB_Y);

                EnsureSize(ref nextX, nextLen);
                EnsureSize(ref nextY, nextLen);

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
                    _velDx[order] = _strengths[order] * wx / ws;
                    _velDy[order] = _strengths[order] * wy / ws;
                }

                curX = nextX;
                curY = nextY;
                curLen = nextLen;
            }

            // Normal acceleration (signed curvature × speed²) for turning
            int naLen = n - 2;
            if (naLen <= 0) return;
            EnsureSize(ref _na, naLen);
            for (int i = 0; i < naLen; i++)
            {
                float ax = _vx[i], ay = _vy[i];
                float bx = _vx[i + 1], by = _vy[i + 1];
                float na_ = ax * by - ay * bx; // cross product = signed curvature × speed²
                float speed = MathF.Sqrt(ax * ax + ay * ay);
                _na[i] = speed > 1e-6f ? na_ / speed : 0f; // normalise to angular velocity
            }

            float wna = 0f, wnaSum = 0f;
            for (int i = 0; i < naLen; i++)
            {
                float w = (i + 1f)*(i + 1f);
                wna += _na[i] * w;
                wnaSum += w;
            }
            _baseNA = wnaSum > 0f ? wna / wnaSum : 0f;

            // Derivative tower for normal acceleration
            var curNa = _na;
            int curNaLen = naLen;

            for (int order = 0; order < 5; order++)
            {
                int nextLen = curNaLen - 1;
                if (nextLen <= 0) break;


                ref float[] nextNa = ref (order % 2 == 0 ? ref _nextNa_A : ref _nextNa_B);
                EnsureSize(ref nextNa, nextLen);

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
                    _naDx[order] = _strengths[order] * wn / ws;

                curNa = nextNa;
                curNaLen = nextLen;
            }

            float naScale = Math.Clamp((float)(LookAheadMs / 50.0), 0f, 1f);
            _baseNA *= naScale;
            for (int i = 0; i < 5; i++) _naDx[i] *= naScale;
        }


        private (float dx, float dy) PredictDelta(double elapsedMs)
        {
        bool dirty;
        lock (_historyLock)
        {
            dirty = _historyDirty;
            if (dirty)
            {
                _historySnapshot.Clear();
                _historySnapshot.AddRange(_history);
                _historyDirty = false;
            }
        }

        if (dirty)
        {
            RebuildCache();
        }

    
    // Integrate predicted path
    double totalT = (LookAheadMs + elapsedMs) / 1000.0;
    float dt = (float)totalT / integrationsteps;
    float dx = 0f, dy = 0f;
    float cvx = _wvx, cvy = _wvy;

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
            dvx += _velDx[order] * tPow / factorial;
            dvy += _velDy[order] * tPow / factorial;
        }

        float rvx = cvx + dvx;
        float rvy = cvy + dvy;

        // Taylor expansion of normal acceleration at this step
        float normalAcc = _baseNA;
        factorial = 1f;
        for (int order = 0; order < 5; order++)
        {
            factorial *= (order + 1);
            float tPow = MathF.Pow(stepT, order + 1);
            normalAcc += _naDx[order] * tPow / factorial;
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
        cvx += _velDx[0] * dt;
        cvy += _velDy[0] * dt;
    }

    return (dx, dy);
}

        public override PipelinePosition Position => PipelinePosition.PostTransform;
    }
}
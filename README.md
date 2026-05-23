# otd-frenet-serret-frame-predictor
OnTake's Frenet-Serret Frame Predictor for OpenTabletDriver

<img width="2560" height="720" alt="frenet-serret-demo" src="https://github.com/user-attachments/assets/b727ec38-1a93-4f31-b90d-e5718b4e05d3" />

# Text version of the banner
## How does this plugin work?  
5 steps!
 - First it'll get the last few reports from your tablet
 - Then it'll fit a cubic B-spline through them to smooth them out
 - Finally it'll compute tangential and normal acceleration over that timespan in a Frenet-Serret reference frame linked to the cursor and its past trajectory (pretty much just determining direction of movement and signed curvature).
 - Then it'll compute a average of the derivatives of those accelerations over that period giving more weight to the recent samples.
 - Then it'll just integrate with those derivative values to compute a prediction of where the Frenet-Serret frame should be at a set offset in the future.


Therefore it will make your tablet seem less sluggish at the expense of added jitter!

## HPR rocket flight computer is using 

### Window filter implementation for detecting states

1. High-G Accelerometer
high-g accelerometer data in addition to regular accelerometer data (he uses two accelerometers) this more precise high Hz moving window is used to detect state changes.

```C
// Moving average calculation (simplified)
float filteredHighG = 0;
for(int i = 0; i < 30; i++) {
    filteredHighG += highGfilter[i];
}
filteredHighG /= 30;
```

2. Smooth Altitude Window
There is implemented a moving window for altitude too 

```C
// Low-pass filter implementation (typical)
float alpha = 0.1; // smoothing factor
baro.smoothAlt = alpha * baro.Alt + (1.0 - alpha) * baro.smoothAlt_prev;

// Or moving average (if implemented)
float altSum = 0;
for(int i = 0; i < filterSize; i++) {
    altSum += altitudeBuffer[i];
}
baro.smoothAlt = altSum / filterSize;
```

### Extra information being collected 

Velocity could be calculated as well as position from acceleration - independent from GPS.
```
vx = ax * t;
sx += vx * t;
```
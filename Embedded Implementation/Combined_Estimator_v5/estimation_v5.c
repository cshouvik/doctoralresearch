#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <math.h>
#include "data.h"

void *estMyf()
{

    float sdelta, sVx, sYawR, sMyr;

    // Initializing UKF
    float P = pMyf;    // Initializing Covariance
    float ns = 3;      // 2*number of states(n)+1
    float R = 0.1;     // Measurement noise covarience
    float Q = 0.1;     // Process noise covarience
    float alpha = 0.1; // Tuning parameter
    float beta = 1;    // Tuning parameter
    float kappa = 30;  // Tuning parameter
    float lambda = alpha * alpha * (1 + kappa) - 1;
    float x = MyfEst; // Initializing state vector
    float Wm_1 = lambda / (1 + lambda);
    float Wm_2 = 0.5 / (1 + lambda);
    float Wm_3 = Wm_2;
    float Wc_1 = Wm_1 + 1 - alpha * alpha + beta;
    float Wc_2 = Wm_2;
    float Wc_3 = Wm_3;
    float r, tDx, Xs_1, Xs_2, Xs_3, Xmean, Xdiff_1, Xdiff_2, Xdiff_3, Pxx, q, var, Ys_1, Ys_2, Ys_3, Ymean, Ydiff_1, Ydiff_2, Ydiff_3, Pyy, Pxy, meas, innovation, K;

    // UKF Algorithm
    // Sigma Point Cakculation
    float r1 = ((1 + lambda) * P);
    float r2 = abs(r1);

    r = sqrt(r2);
    // Time Update
    float alpha_1 = lr * m * Vx_curr / (l * cos(delta_curr));
    tDx = Tsample * (alpha_1 * BetaR_prev + alpha_1 * YawR_prev + alpha_1 * (Iz / (lr * m * Vx_curr)) * YawAcc_prev); //x_t=x_t-1+tDx;
    Xs_1 = x + tDx;
    Xs_2 = x + r + tDx;
    Xs_3 = x - r + tDx;

    Xmean = Wm_1 * Xs_1 + Wm_2 * Xs_2 + Wm_3 * Xs_3;

    Xdiff_1 = Xs_1 - Xmean;
    Xdiff_2 = Xs_2 - Xmean;
    Xdiff_3 = Xs_3 - Xmean;

    Pxx = Wc_1 * Xdiff_1 * Xdiff_1 + Wc_2 * Xdiff_2 * Xdiff_2 + Wc_3 * Xdiff_3 * Xdiff_3 + Q;
    // Resampling of Sigma Points
    float q1 = (1 + lambda) * Pxx;
    float q2 = abs(q1);
    q = sqrt(q2);
    //Measurement Update(Measurement=>Ay)
    var = YawR_curr * Vx_curr + (1 / m) * Fyr_curr;                       //Unpropagated Term
    Ys_1 = var - (1 / m) * cos(delta_curr) * ((Xmean - x) / Tsample);     //Fyf=((Xmean-x)/Tsample);
    Ys_2 = var - (1 / m) * cos(delta_curr) * ((Xmean + q - x) / Tsample); //Fyf=((Xmean-x)/Tsample);
    Ys_3 = var - (1 / m) * cos(delta_curr) * ((Xmean - q - x) / Tsample); //Fyf=((Xmean-x)/Tsample);
    Ymean = Wc_1 * Ys_1 + Wc_2 * Ys_2 + Wc_3 * Ys_3;

    Ydiff_1 = Ys_1 - Ymean;
    Ydiff_2 = Ys_2 - Ymean;
    Ydiff_3 = Ys_3 - Ymean;
    Pyy = Wc_1 * Ydiff_1 * Ydiff_1 + Wc_2 * Ydiff_2 * Ydiff_2 + Wc_3 * Ydiff_3 * Ydiff_3 + R;

    Pxy = Wc_1 * Ydiff_1 * Xdiff_1 + Wc_2 * Ydiff_2 * Xdiff_2 + Wc_3 * Ydiff_3 * Xdiff_3;
    // Kalman Gain and Innovation
    meas = Ay_curr; // Temporarily changing Ay to Vx
    K = Pxy / Pyy;
    innovation = meas - Ymean;
    x = Xmean + K * innovation;
    P = Pxx - K * K * Pyy;

    MyfEst = x;
    pMyf = P;

    //printf("Completing thread_1 Execution\n");

    return NULL;
}

void *estMyr()
{

    float sdelta, sVx, sYawR, sMyr;
    // Initializing UKF
    //float Tsample = Time[2] - Time[1];    // Sampling Time
    float P = pMyr;    // Initializing Covariance
    float ns = 3;      // 2*number of states(n)+1
    float R = 0.1;     // Measurement noise covarience
    float Q = 0.1;     // Process noise covarience
    float alpha = 0.1; // Tuning parameter
    float beta = 1;    // Tuning parameter
    float kappa = 30;  // Tuning parameter
    float lambda = alpha * alpha * (1 + kappa) - 1;
    float x = MyrEst; // Initializing state vector
    float Wm_1 = lambda / (1 + lambda);
    float Wm_2 = 0.5 / (1 + lambda);
    float Wm_3 = Wm_2;
    float Wc_1 = Wm_1 + 1 - alpha * alpha + beta;
    float Wc_2 = Wm_2;
    float Wc_3 = Wm_3;
    float r, tDx, Xs_1, Xs_2, Xs_3, Xmean, Xdiff_1, Xdiff_2, Xdiff_3, Pxx, q, var, Ys_1, Ys_2, Ys_3, Ymean, Ydiff_1, Ydiff_2, Ydiff_3, Pyy, Pxy, meas, innovation, K;

    // UKF Algorithm
    // Sigma Point Cakculation
    float r1 = ((1 + lambda) * P);
    float r2 = abs(r1);

    r = sqrt(r2);
    // Time Update
    tDx = Tsample * ((lf * m * Vx_prev / l) * BetaR_prev + (lf * m * Vx_prev / l) * YawR_prev - (Iz / l) * YawAcc_prev); //x_t=x_t-1+tDx;
    Xs_1 = x + tDx;
    Xs_2 = x + r + tDx;
    Xs_3 = x - r + tDx;

    Xmean = Wm_1 * Xs_1 + Wm_2 * Xs_2 + Wm_3 * Xs_3;

    Xdiff_1 = Xs_1 - Xmean;
    Xdiff_2 = Xs_2 - Xmean;
    Xdiff_3 = Xs_3 - Xmean;

    Pxx = Wc_1 * Xdiff_1 * Xdiff_1 + Wc_2 * Xdiff_2 * Xdiff_2 + Wc_3 * Xdiff_3 * Xdiff_3 + Q;
    // Resampling of Sigma Points
    float q1 = (1 + lambda) * Pxx;
    float q2 = abs(q1);
    q = sqrt(q2);

    //Measurement Update(Measurement=>Ay)
    var = YawR_curr * Vx_curr - (1 / m) * Fyf_curr * cos(delta_curr); //Unpropagated Term
    Ys_1 = var + (1 / m) * ((Xmean - x) / Tsample);                   //Fyr=((Xmean-x)/Tsample);
    Ys_2 = var + (1 / m) * ((Xmean + q - x) / Tsample);               //Fyr=((Xmean-x)/Tsample);
    Ys_3 = var + (1 / m) * ((Xmean - q - x) / Tsample);               //Fyr=((Xmean-x)/Tsample);

    Ymean = Wc_1 * Ys_1 + Wc_2 * Ys_2 + Wc_3 * Ys_3;

    Ydiff_1 = Ys_1 - Ymean;
    Ydiff_2 = Ys_2 - Ymean;
    Ydiff_3 = Ys_3 - Ymean;

    Pyy = Wc_1 * Ydiff_1 * Ydiff_1 + Wc_2 * Ydiff_2 * Ydiff_2 + Wc_3 * Ydiff_3 * Ydiff_3;

    Pxy = Wc_1 * Ydiff_1 * Xdiff_1 + Wc_2 * Ydiff_2 * Xdiff_2 + Wc_3 * Ydiff_3 * Xdiff_3;
    // Kalman Gain and Innovation
    meas = Ay_curr; // Temporarily changing Ay to Vx
    K = Pxy / Pyy;
    innovation = meas - Ymean;

    x = Xmean + K * innovation;
    P = Pxx - K * K * Pyy;

    MyrEst = x;
    pMyr = P;

    //printf("Completing thread_2 Execution\n");
    return NULL;
}

void *estYawR()
{

    float sdelta, sVx, sYawR, sMyr, sYawREst, smeasYawR; //variables for serial print
    // Initializing UKF
    //float Tsample = Time[2] - Time[1];    // Sampling Time
    float P = pYawR;       // Initializing Covariance
    float ns = 3;          // 2*number of states(n)+1
    float R = 100;         // Measurement noise covarience
    float Q = 0.01;        // Process noise covarience
    float alpha = 0.1;     // Tuning parameter
    float beta = 10;       // Tuning parameter
    float kappa = rYawEst; // Tuning parameter
    float lambda = alpha * alpha * (1 + kappa) - 1;
    float x = rYawEst; // Initializing state vector
    float Wm_1 = lambda / (1 + lambda);
    float Wm_2 = 0.5 / (1 + lambda);
    float Wm_3 = Wm_2;
    float Wc_1 = Wm_1 + 1 - alpha * alpha + beta;
    float Wc_2 = Wm_2;
    float Wc_3 = Wm_3;
    float r, tDx, Xs_1, Xs_2, Xs_3, Xmean, Xdiff_1, Xdiff_2, Xdiff_3, Pxx, q, var, Ys_1, Ys_2, Ys_3, Ymean, Ydiff_1, Ydiff_2, Ydiff_3, Pyy, Pxy, meas, innovation, K;

    float steerdegree = abs(delta_curr) * 57.2958; //Trigger condition introduced as extensive zero signals are causing NaN output.
    if (steerdegree >= 0)
    {
        // UKF Algorithm
        // Sigma Point Cakculation
        float r1 = ((1 + lambda) * P);
        float r2 = abs(r1);
        r = sqrt(r2);

        // Time Update
        tDx = Tsample * (1 / Iz) * (lf * Fyf_prev * cos(delta_prev) - lr * Fyr_prev); //x_t=x_t-1+tDx;
        Xs_1 = x + tDx;
        Xs_2 = x + r + tDx;
        Xs_3 = x - r + tDx;

        Xmean = Wm_1 * Xs_1 + Wm_2 * Xs_2 + Wm_3 * Xs_3;

        Xdiff_1 = Xs_1 - Xmean;
        Xdiff_2 = Xs_2 - Xmean;
        Xdiff_3 = Xs_3 - Xmean;

        Pxx = Wc_1 * Xdiff_1 * Xdiff_1 + Wc_2 * Xdiff_2 * Xdiff_2 + Wc_3 * Xdiff_3 * Xdiff_3 + Q;
        // Resampling of Sigma Points
        float q1 = (1 + lambda) * Pxx;
        float q2 = abs(q1);
        q = sqrt(q2);

        //Measurement Update(Measurement=>YawR)
        Ys_1 = Xmean;
        Ys_2 = Xmean + q;
        Ys_3 = Xmean - q;

        Ymean = Wc_1 * Ys_1 + Wc_2 * Ys_2 + Wc_3 * Ys_3;

        Ydiff_1 = Ys_1 - Ymean;
        Ydiff_2 = Ys_2 - Ymean;
        Ydiff_3 = Ys_3 - Ymean;

        Pyy = Wc_1 * Ydiff_1 * Ydiff_1 + Wc_2 * Ydiff_2 * Ydiff_2 + Wc_3 * Ydiff_3 * Ydiff_3 + R;

        Pxy = Wc_1 * Ydiff_1 * Xdiff_1 + Wc_2 * Ydiff_2 * Xdiff_2 + Wc_3 * Ydiff_3 * Xdiff_3;

        // Kalman Gain and Innovation
        meas = measYawR_curr;
        K = Pxy / Pyy;
        innovation = meas - Ymean;

        x = Xmean + K * innovation;
        P = Pxx - K * K * Pyy;
    }
    rYawEst = x;
    pYawR = P;
    //printf("Completing thread_3 Execution\n");
    return NULL;
}

void *estBeta()
{
    // Initializing UKF
    //float Tsample = Time[2] - Time[1];    // Sampling Time
    float P = pBeta;   // Initializing Covariance
    float ns = 3;      // 2*number of states(n)+1
    float R = 10;      // Measurement noise covarience
    float Q = 0.005;   // Process noise covarience
    float alpha = 1.4; // Tuning parameter
    float beta = 1;    // Tuning parameter
    float kappa = 1;   // Tuning parameter
    float lambda = alpha * alpha * (1 + kappa) - 1;
    float x = betaEst; // Initializing state vector
    float Wm_1 = lambda / (1 + lambda);
    float Wm_2 = 0.5 / (1 + lambda);
    float Wm_3 = Wm_2;
    float Wc_1 = Wm_1 + 1 - alpha * alpha + beta;
    float Wc_2 = Wm_2;
    float Wc_3 = Wm_3;
    float r, tDx, Xs_1, Xs_2, Xs_3, Xmean, Xdiff_1, Xdiff_2, Xdiff_3, Pxx, q, var, Ys_1, Ys_2, Ys_3, Ymean, Ydiff_1, Ydiff_2, Ydiff_3, Pyy, Pxy, meas, innovation, K;
    //float steerdegree = abs(delta_curr) * 57.2958; //Trigger condition introduced as extensive zero signals are causing NaN output.
    if (delta_curr != 0)
    {
        // UKF Algorithm
        // Sigma Point Cakculation
        float r1 = ((1 + lambda) * P);
        float r2 = abs(r1);
        r = sqrt(r2);

        // Time Update
        tDx = Tsample * ((1 / (m * Vx_prev)) * (Fyf_prev * cos(delta_prev) + Fyr_prev) - YawR_prev) + m * g * sin(-phi_Prev); //NB: As Vcg=sqrt(Vx.^2+Vy.^2) and Vx^2>>Vy^2 hence Vcg //x_t=x_t-1+tDx;
        Xs_1 = x + tDx;
        Xs_2 = x + r + tDx;
        Xs_3 = x - r + tDx;

        Xmean = Wm_1 * Xs_1 + Wm_2 * Xs_2 + Wm_3 * Xs_3;

        Xdiff_1 = Xs_1 - Xmean;
        Xdiff_2 = Xs_2 - Xmean;
        Xdiff_3 = Xs_3 - Xmean;

        Pxx = Wc_1 * Xdiff_1 * Xdiff_1 + Wc_2 * Xdiff_2 * Xdiff_2 + Wc_3 * Xdiff_3 * Xdiff_3 + Q;
        // Resampling of Sigma Points
        float q1 = (1 + lambda) * Pxx;
        float q2 = abs(q1);
        q = sqrt(q2);

        //Measurement Update(Measurement=Vy_pseudo)
        Ys_1 = Vx_curr * tan(Xmean);
        Ys_2 = Vx_curr * tan(Xmean + q);
        Ys_3 = Vx_curr * tan(Xmean - q);

        Ymean = Wc_1 * Ys_1 + Wc_2 * Ys_2 + Wc_3 * Ys_3;

        Ydiff_1 = Ys_1 - Ymean;
        Ydiff_2 = Ys_2 - Ymean;
        Ydiff_3 = Ys_3 - Ymean;

        Pyy = Wc_1 * Ydiff_1 * Ydiff_1 + Wc_2 * Ydiff_2 * Ydiff_2 + Wc_3 * Ydiff_3 * Ydiff_3 + R;

        Pxy = Wc_1 * Ydiff_1 * Xdiff_1 + Wc_2 * Ydiff_2 * Xdiff_2 + Wc_3 * Ydiff_3 * Xdiff_3;
        // Kalman Gain and Innovation
        meas = measVy_curr;
        K = Pxy / Pyy;
        innovation = meas - Ymean;

        x = Xmean + K * innovation;
        P = Pxx - K * K * Pyy;
    }
    betaEst = x;
    pBeta = P;
    return NULL;
}

void *estVy()
{
    float P = pVy;     // Initializing Covariance
    float ns = 3;      // 2*number of states(n)+1
    float R = 100;     // Measurement noise covarience
    float Q = 0.001;   // Process noise covarience
    float alpha = 1.2; // Tuning parameter
    float beta = 0.1;  // Tuning parameter
    float kappa = 0;   // Tuning parameter
    float lambda = alpha * alpha * (1 + kappa) - 1;
    float x = VyEst; // Initializing state vector
    float Wm_1 = lambda / (1 + lambda);
    float Wm_2 = 0.5 / (1 + lambda);
    float Wm_3 = Wm_2;
    float Wc_1 = Wm_1 + 1 - alpha * alpha + beta;
    float Wc_2 = Wm_2;
    float Wc_3 = Wm_3;
    float r, tDx, Xs_1, Xs_2, Xs_3, Xmean, Xdiff_1, Xdiff_2, Xdiff_3, Pxx, q, var, Ys_1, Ys_2, Ys_3, Ymean, Ydiff_1, Ydiff_2, Ydiff_3, Pyy, Pxy, meas, innovation, K;

    if (delta_curr != 0)
    {
        // UKF Algorithm
        // Sigma Point Cakculation
        float r1 = ((1 + lambda) * P);
        float r2 = abs(r1);
        float r = sqrt(r2);

        // Time Update
        float phicos = cos(phi_Prev);
        float phitan = tan(phi_Prev);
        float tdx_c1 = (Vx_prev / phicos) * BetaR_prev;
        float tdx_c2 = Vx_prev * ((1 / phicos) - 1) * YawR_prev;
        float tdx_c3 = (g / Vx_prev) * phitan;
        float tDx = Tsample * (tdx_c1 + tdx_c2 - tdx_c3); //x_t=x_t-1+tDx;
        Xs_1 = x + tDx;
        Xs_2 = x + r + tDx;
        Xs_3 = x - r + tDx;

        Xmean = Wm_1 * Xs_1 + Wm_2 * Xs_2 + Wm_3 * Xs_3;

        Xdiff_1 = Xs_1 - Xmean;
        Xdiff_2 = Xs_2 - Xmean;
        Xdiff_3 = Xs_3 - Xmean;

        Pxx = Wc_1 * Xdiff_1 * Xdiff_1 + Wc_2 * Xdiff_2 * Xdiff_2 + Wc_3 * Xdiff_3 * Xdiff_3 + Q;
        // Resampling of Sigma Points
        float q1 = (1 + lambda) * Pxx;
        float q2 = abs(q1);
        q = sqrt(q2);

        //Measurement Update(Measurement=>Vy=Vx*Beta;)
        Ys_1 = Xmean;
        Ys_2 = Xmean + q;
        Ys_3 = Xmean - q;

        Ymean = Wc_1 * Ys_1 + Wc_2 * Ys_2 + Wc_3 * Ys_3;

        Ydiff_1 = Ys_1 - Ymean;
        Ydiff_2 = Ys_2 - Ymean;
        Ydiff_3 = Ys_3 - Ymean;

        Pyy = Wc_1 * Ydiff_1 * Ydiff_1 + Wc_2 * Ydiff_2 * Ydiff_2 + Wc_3 * Ydiff_3 * Ydiff_3 + R;

        Pxy = Wc_1 * Ydiff_1 * Xdiff_1 + Wc_2 * Ydiff_2 * Xdiff_2 + Wc_3 * Ydiff_3 * Xdiff_3;
        // Kalman Gain and Innovation
        meas = Vx_curr * Beta_prev;
        K = Pxy / Pyy;
        innovation = meas - Ymean;

        x = Xmean + K * innovation;
        P = Pxx - K * K * Pyy;
    }
    VyEst = x;
    pVy = P;
    return NULL;
}
int updateModule()
{

    pthread_t thread_1, thread_2, thread_3, thread_4, thread_5;
    pthread_create(&thread_1, NULL, estMyf, NULL);
    pthread_create(&thread_2, NULL, estMyr, NULL);
    pthread_create(&thread_3, NULL, estYawR, NULL);
    pthread_create(&thread_4, NULL, estBeta, NULL);
    pthread_create(&thread_5, NULL, estVy, NULL);
    pthread_join(thread_1, NULL);
    pthread_join(thread_2, NULL);
    pthread_join(thread_3, NULL);
    pthread_join(thread_4, NULL);
    pthread_join(thread_5, NULL);

    return 0;
}

int dataHandleModule(int runs)
{

    if (runs == 1)
    {

        // Previous time variable
        delta_prev = 0;
        Vx_prev = 0;
        Fyf_prev = 0;
        Fyr_prev = 0;
        Beta_prev = 0;
        BetaR_prev = 0;
        YawR_prev = 0;
        YawAcc_prev = 0;
        phi_Prev = 0;
    }

    if (runs > 1)
    {

        // Previous time variable
        delta_prev = delta_curr;
        Vx_prev = Vx_curr;
        Fyf_prev = Fyf_curr;
        Fyr_prev = Fyr_curr;
        Beta_prev = Beta_curr;
        BetaR_prev = BetaR_curr;
        YawR_prev = YawR_curr;
        YawAcc_prev = YawAcc_curr;
        phi_Prev = phi_curr;
    }

    delta_curr = delta_n;
    Vx_curr = Vx_n;
    Fyr_curr = Fyr_n;
    Fyf_curr = Fyf_n;
    BetaR_curr = BetaR_n;
    Beta_curr = Beta_n;
    YawR_curr = YawR_n;
    YawAcc_curr = YawAcc_n;
    phi_curr = phi_n;

    Ay_curr = Ay_n;
    measYawR_curr = measYawR_n;
    measVy_curr = measVy_curr_n;

    return 0;
}

int dataCallModule(int i)
{

    delta_n = delta[i];
    Vx_n = Vx[i];
    Fyr_n = Fyr[i];
    Fyf_n = Fyf[i];
    BetaR_n = BetaR[i];
    Beta_n = Beta[i];
    YawR_n = YawR[i];
    YawAcc_n = YawAcc[i];
    phi_n = phi[i];

    Ay_n = Ay[i];
    measYawR_n = measYawR[i];
    measVy_curr_n = Vy[i];

    return 0;
}

int main()
{

    FILE *fptr;

    // open the file in write mode
    fptr = fopen("output.csv", "w");

    if (fptr != NULL)
    {
        printf("File created successfully!\n");
    }
    else
    {
        printf("Failed to create the file.\n");
        // exit status for OS that an error occured
        return -1;
    }

    fprintf(fptr, "%s %s %s %s %s %s %s %s %s %s %s %s %s %s\n", "delta", "Vx", "Vy", "Time", "Fyf", "MyfBuf", "Fyr", "MyrBuf", "YawR", "rYawBuf", "Beta", "BetaBuf", "Vy", "VyBuf");

    for (int i = 1; i < 321; i++)
    {

        run++;

        dataCallModule(i);

        dataHandleModule(run);

        updateModule();

        MyfBuf[i] = MyfEst;
        MyrBuf[i] = MyrEst;
        rYawBuf[i] = rYawEst;
        BetaBuf[i] = betaEst;
        VyBuf[i] = VyEst;
        deltaBuf[i] = delta_curr;

        //printf("Execution number: %d\n", i);

        // write data in file
        fprintf(fptr, "%9.6f %9.6f %9.6f %9.6f %9.6f %9.6f %9.6f %9.6f %9.6f %9.6f %9.6f %9.6f %9.6f %9.6f \n", deltaBuf[i], Vx[i], Vy[i], Time[i], Fyf[i], MyfBuf[i], Fyr[i], MyrBuf[i], YawR[i], rYawBuf[i], Beta[i], BetaBuf[i], Vy[i], VyBuf[i]);

        // close connection
    }

    fclose(fptr);
    //int ticks = clock()/4;
    //printf("Execution time is %f\n", (float)ticks/CLOCKS_PER_SEC);

    return 0;
}
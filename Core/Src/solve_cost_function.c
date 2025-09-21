/*
 * solve_cost_function.c
 *
 *  Created on: Sep 7, 2025
 *      Author: gerrygeyer
 */


// lqr_h7.c  (einfach in Core/Src; Header unten)
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <solve_cost_function.h>
#include <parameter.h>
#include <sys_math.h>



// --- State ---
typedef enum { CF_IDLE=0, CF_BUSY, CF_READY } cf_state_t;
static volatile cf_state_t cf_state = CF_IDLE;

static volatile float Qdiag_mem[6];
static volatile float Rdiag_mem[3];
static volatile float Ts_mem;

static int16_t K_q10_bufA[K_ROWS][K_COLS];
static int16_t K_q10_bufB[K_ROWS][K_COLS];
static volatile uint32_t K_version = 0;     // even=A, odd=B
static volatile uint8_t  K_ready = 0;

// --- helpers ---
static inline int16_t q10_round_sat(float v){
    float s = v * Q10;
    // symmetrisches Runden (ties to away from zero)
    s = (s >= 0.0f) ? floorf(s + 0.5f) : ceilf(s - 0.5f);
    if (s >  32767.0f) return  32767;
    if (s < -32768.0f) return -32768;
    return (int16_t)s;
}

static void convertK_q10(const float *K, int16_t dst[K_ROWS][K_COLS]){
    for (uint8_t i=0;i<K_ROWS;i++)
        for (uint8_t j=0;j<K_COLS;j++)
            dst[i][j] = q10_round_sat(K[i*6 + j]);
}

// --- API: Parameter setzen + Trigger ---
void run_cost_fct(float q1i,float q2i,float q3i,float q4i,float q5i,float q6i,
                  float r1i,float r2i,float r3i, float Ts_input)
{
    if (cf_state != CF_IDLE) return;        // schon in Arbeit oder bereit
    Qdiag_mem[0]=q1i; Qdiag_mem[1]=q2i; Qdiag_mem[2]=q3i;
    Qdiag_mem[3]=q4i; Qdiag_mem[4]=q5i; Qdiag_mem[5]=q6i;
    Rdiag_mem[0]=r1i; Rdiag_mem[1]=r2i; Rdiag_mem[2]=r3i;
    Ts_mem = Ts_input;
    K_ready = 0;
    cf_state = CF_BUSY;                      // Trigger
}

// --- Wird zyklisch in while(1) aufgerufen (niedrige Prio/IT-Modus kompatibel) ---
void run_cost_function(void)
{
    if (cf_state != CF_BUSY) return;

    // Snapshot der Eingaben (atomar genug auf Cortex-M, da float write/read 32-bit)
    float Qdloc[6], Rdloc[3];
    for (uint8_t i=0;i<6;i++) Qdloc[i] = (float)Qdiag_mem[i];
    for (uint8_t i=0;i<3;i++) Rdloc[i] = (float)Rdiag_mem[i];
    double Ts = (double)Ts_mem;
    if (!(Ts > 0.0)) { cf_state = CF_IDLE; return; }

    // Matritzen
    float Ad[36], Bd[18], Qd[36], Rd[9], K[18], P[36];
    build_Ad_Bd(Ts, Ad, Bd);
    build_Qd_Rd_scaled(Ts, Qdloc, Rdloc, Qd, Rd, true);

    // Rechnen (nicht-blockierend aus Sicht Hauptloop; hier synchron in dieser Funktion)
    bool ok = lqr_dare_iter2x2(Ad,Bd,Qd,Rd, K,P, COST_FCT_TOLEARANCE, COST_FCT_ITERATIONS);

    // Ergebnis -> freien Buffer
    int16_t (*dst)[K_COLS] = ((K_version & 1u)==0u) ? K_q10_bufB : K_q10_bufA;
    if (ok) convertK_q10(K, dst); else { cf_state = CF_IDLE; return; }

    // Sichtbar machen: Version flippen, READY setzen
    __disable_irq();
    K_version++;
    K_ready = 1;
    cf_state = CF_READY;
    __enable_irq();
}

// --- Konsument: K abrufen (liefert 1 bei Erfolg) ---
uint8_t getK_matrix(int16_t Kout[K_ROWS][K_COLS])
{
    if (!K_ready) return 0;
    __disable_irq();
    uint32_t ver = K_version;
    int16_t (*src)[K_COLS] = ((ver & 1u)==0u) ? K_q10_bufA : K_q10_bufB;
    for (uint8_t i=0;i<K_ROWS;i++)
        for (uint8_t j=0;j<K_COLS;j++)
            Kout[i][j] = src[i][j];
    // optional: nach Abholen wieder freigeben
    K_ready = 0;
    cf_state = CF_IDLE;
    __enable_irq();
    return 1;
}


// ---- kleine Helfer für feste Dimensionen ----
static inline void mat6x6_T(const float A[36], float AT[36]) {
    for (int r=0;r<6;r++) for (int c=0;c<6;c++) AT[c*6+r]=A[r*6+c];
}
static inline void mat6x3_T(const float B[18], float BT[18]) {
    for (int r=0;r<6;r++) for (int c=0;c<3;c++) BT[c*6+r]=B[r*3+c];
}
static inline void mat6x6_mul6x6(const float A[36], const float B[36], float C[36]) {
    for(int r=0;r<6;r++){ for(int c=0;c<6;c++){ float s=0;
        for(int k=0;k<6;k++) s+=A[r*6+k]*B[k*6+c]; C[r*6+c]=s; } }
}
static inline void mat6x6_mul6x3(const float A[36], const float B[18], float C[18]) {
    for(int r=0;r<6;r++){ for(int c=0;c<3;c++){ float s=0;
        for(int k=0;k<6;k++) s+=A[r*6+k]*B[k*3+c]; C[r*3+c]=s; } }
}
static inline void mat6x3_mul3x6(const float A[18], const float B[18], float C[36]) {
    for(int r=0;r<6;r++){ for(int c=0;c<6;c++){ float s=0;
        for(int k=0;k<3;k++) s+=A[r*3+k]*B[k*6+c]; C[r*6+c]=s; } }
}
static inline void mat3x6_mul6x6(const float A[18], const float B[36], float C[18]) {
    for(int r=0;r<3;r++){ for(int c=0;c<6;c++){ float s=0;
        for(int k=0;k<6;k++) s+=A[r*6+k]*B[k*6+c]; C[r*6+c]=s; } }
}
static inline void mat3x6_mul6x3(const float A[18], const float B[18], float C[9]) {
    for(int r=0;r<3;r++){ for(int c=0;c<3;c++){ float s=0;
        for(int k=0;k<6;k++) s+=A[r*6+k]*B[k*3+c]; C[r*3+c]=s; } }
}
static inline void mat6x6_add(const float A[36], const float B[36], float C[36]) {
    for(int i=0;i<36;i++) C[i]=A[i]+B[i];
}
static inline void mat6x6_sub(const float A[36], const float B[36], float C[36]) {
    for(int i=0;i<36;i++) C[i]=A[i]-B[i];
}
static inline void symmetrize6(float P[36]) { // numerisch stabiler
    for(int r=0;r<6;r++) for(int c=r+1;c<6;c++){
        float s=0.5f*(P[r*6+c]+P[c*6+r]); P[r*6+c]=P[c*6+r]=s;
    }
}
static inline float inf_norm6x6_diff(const float A[36], const float B[36]) {
    float m=0; for(int i=0;i<36;i++){ float d=fabsf(A[i]-B[i]); if(d>m) m=d; } return m;
}

// ---- 3x3 Cholesky + LGS-Löser (SPD) ----
static bool chol3(const float M[9], float L[9]) {
    // M = L L^T, untere Dreiecksmatrix in L
    float l00 = sqrtf(M[0]); if(!(l00>0)) return false;
    float l10 = M[3]/l00;
    float l20 = M[6]/l00;
    float l11 = sqrtf(M[4]-l10*l10); if(!(l11>0)) return false;
    float l21 = (M[7]-l10*l20)/l11;
    float l22 = sqrtf(M[8]-l20*l20-l21*l21); if(!(l22>0)) return false;
    memset(L,0,9*sizeof(float));
    L[0]=l00; L[3]=l10; L[6]=l20; L[4]=l11; L[7]=l21; L[8]=l22;
    return true;
}
static void chol3_solve(const float L[9], const float b[3], float x[3]) {
    // vorwärts: L y = b
    float y0 = b[0]/L[0];
    float y1 = (b[1]-L[3]*y0)/L[4];
    float y2 = (b[2]-L[6]*y0-L[7]*y1)/L[8];
    // rückwärts: L^T x = y
    float x2 = y2/L[8];
    float x1 = (y1 - L[7]*x2)/L[4];
    float x0 = (y0 - L[3]*x1 - L[6]*x2)/L[0];
    x[0]=x0; x[1]=x1; x[2]=x2;
}

void build_Qd_Rd_scaled(double Ts,
                        const float Qdiag[6], const float Rdiag[3],
                        float Qd[36], float Rd[9],
                        bool scale_by_Ts)
{
    memset(Qd, 0, 36*sizeof(float));
    memset(Rd, 0,  9*sizeof(float));
    float sQ = scale_by_Ts ? (float)Ts : 1.0f;
    float sR = scale_by_Ts ? (float)Ts : 1.0f;
    for (int i=0;i<6;i++) Qd[i*6+i] = sQ * Qdiag[i];
    for (int i=0;i<3;i++) Rd[i*3+i] = sR * Rdiag[i];
}

// ---- Ad, Bd für 3x Doppelintegrator ----
void build_Ad_Bd(double Ts, float Ad[36], float Bd[18]) {
    memset(Ad,0,36*sizeof(float));
    memset(Bd,0,18*sizeof(float));
    // Ad = [I Ts*I; 0 I], Bd = [0.5*Ts^2*I; Ts*I]
    for(int i=0;i<3;i++){
        Ad[i*6 + i] = 1.0f;               // I (pos)
        Ad[(i+3)*6 + (i+3)] = 1.0f;       // I (vel)
        Ad[i*6 + (i+3)] = (float)Ts;      // Ts
        Bd[i*3 + i] = 0.5f*(float)(Ts*Ts);
        Bd[(i+3)*3 + i] = (float)Ts;
    }
}

// ---- Qd, Rd aus Diagonalen (Riemann-Summen-Skalierung) ----
void build_Qd_Rd(double Ts, const float Qdiag[6], const float Rdiag[3],
                 float Qd[36], float Rd[9]) {
    memset(Qd,0,36*sizeof(float));
    memset(Rd,0, 9*sizeof(float));
    float s = (float)Ts;
    for(int i=0;i<6;i++) Qd[i*6+i] = s * Qdiag[i];
    for(int i=0;i<3;i++) Rd[i*3+i] = s * Rdiag[i];
}

// ---- DARE via Riccati-Iteration; Ergebnis: K (3x6) & P (6x6) ----
bool lqr_dare_iter(const float Ad[36], const float Bd[18],
                   const float Qd[36], const float Rd[9],
                   float K[18], float Pout[36],
                   float tol, int max_it)
{
    // Kopien nach double
    double Ad_d[36], Bd_d[18], Qd_d[36], Rd_d[9];
    for (int i=0;i<36;i++){ Ad_d[i]=(double)Ad[i]; Qd_d[i]=(double)Qd[i]; }
    for (int i=0;i<18;i++)  Bd_d[i]=(double)Bd[i];
    for (int i=0;i<9;i++)   Rd_d[i]=(double)Rd[i];

    // Transponierte
    double AT[36], BT[18];
    for (int r=0;r<6;r++) for (int c=0;c<6;c++) AT[c*6+r] = Ad_d[r*6+c];
    for (int r=0;r<6;r++) for (int c=0;c<3;c++) BT[c*6+r] = Bd_d[r*3+c];

    // P-Iteration (double)
    double P[36]; for (int i=0;i<36;i++) P[i]=Qd_d[i];
    const double tol_d = (double)tol;

    // lokale Buffer
    double PB[18], BT_P_B[9], M[9], L[9];
    double AP[36], S[18], G[18];
    double PA[36], AT_P_A[36], SG[36], Pnext[36];

    // Cholesky 3x3 (double)
    auto bool chol3_d(const double M_[9], double L_[9]) {
        double l00 = sqrt(M_[0]); if(!(l00>0.0)) return false;
        double l10 = M_[3]/l00, l20 = M_[6]/l00;
        double t11 = M_[4] - l10*l10; if(!(t11>0.0)) return false;
        double l11 = sqrt(t11);
        double l21 = (M_[7]-l10*l20)/l11;
        double s22 = M_[8] - l20*l20 - l21*l21; if(!(s22>0.0)) return false;
        double l22 = sqrt(s22);
        for(int i=0;i<9;i++) L_[i]=0.0;
        L_[0]=l00; L_[3]=l10; L_[6]=l20; L_[4]=l11; L_[7]=l21; L_[8]=l22;
        return true;
    }
    auto void chol3_solve_d(const double L_[9], const double b[3], double x[3]) {
        double y0=b[0]/L_[0];
        double y1=(b[1]-L_[3]*y0)/L_[4];
        double y2=(b[2]-L_[6]*y0-L_[7]*y1)/L_[8];
        x[2]=y2/L_[8];
        x[1]=(y1-L_[7]*x[2])/L_[4];
        x[0]=(y0-L_[3]*x[1]-L_[6]*x[2])/L_[0];
    }

    for (int it=0; it<max_it; ++it) {
        // M = Rd + B^T P B
        for(int r=0;r<6;r++) for(int c=0;c<3;c++){
            double s=0.0; for(int k=0;k<6;k++) s+=P[r*6+k]*Bd_d[k*3+c];
            PB[r*3+c]=s;
        }
        for(int r=0;r<3;r++) for(int c=0;c<3;c++){
            double s=0.0; for(int k=0;k<6;k++) s+=BT[r*6+k]*PB[k*3+c];
            BT_P_B[r*3+c]=s;
        }
        for(int i=0;i<9;i++) M[i]=Rd_d[i]+BT_P_B[i];

        // S = A^T P B
        for(int r=0;r<6;r++) for(int c=0;c<6;c++){
            double s=0.0; for(int k=0;k<6;k++) s+=AT[r*6+k]*P[k*6+c];
            AP[r*6+c]=s;
        }
        for(int r=0;r<6;r++) for(int c=0;c<3;c++){
            double s=0.0; for(int k=0;k<6;k++) s+=AP[r*6+k]*Bd_d[k*3+c];
            S[r*3+c]=s;
        }

        // löse M * G = S^T   (G: 3x6)
        if(!chol3_d(M,L)) return false;
        for(int col=0; col<6; ++col){
            // Zeile 'col' von S (Stride 3!)
            double b[3]={ S[col*3+0], S[col*3+1], S[col*3+2] };
            double x[3]; chol3_solve_d(L,b,x);
            G[0*6+col]=x[0]; G[1*6+col]=x[1]; G[2*6+col]=x[2];
        }

        // Pnext = A^T P A - S*G + Qd
        for(int r=0;r<6;r++) for(int c=0;c<6;c++){
            double s1=0.0; for(int k=0;k<6;k++) s1+=P[r*6+k]*Ad_d[k*6+c];
            PA[r*6+c]=s1; // temporär
        }
        for(int r=0;r<6;r++) for(int c=0;c<6;c++){
            double s=0.0; for(int k=0;k<6;k++) s+=AT[r*6+k]*PA[k*6+c];
            AT_P_A[r*6+c]=s;
        }
        for(int r=0;r<6;r++) for(int c=0;c<6;c++){
            double s=0.0; for(int k=0;k<3;k++) s+=S[r*3+k]*G[k*6+c];
            SG[r*6+c]=s;
        }
        for(int i=0;i<36;i++) Pnext[i]=AT_P_A[i]-SG[i]+Qd_d[i];
        // symmetrize
        for(int r=0;r<6;r++) for(int c=r+1;c<6;c++){
            double s=0.5*(Pnext[r*6+c]+Pnext[c*6+r]);
            Pnext[r*6+c]=s; Pnext[c*6+r]=s;
        }

        // Abbruch
        double dn=0.0; for(int i=0;i<36;i++){ double d=fabs(Pnext[i]-P[i]); if(d>dn) dn=d; }
        for(int i=0;i<36;i++) P[i]=Pnext[i];
        if(dn<tol_d) break;
    }

    // *** WICHTIG: PA jetzt mit FINALem P neu berechnen! ***
    for(int r=0;r<6;r++) for(int c=0;c<6;c++){
        double s=0.0; for(int k=0;k<6;k++) s+=P[r*6+k]*Ad_d[k*6+c];
        PA[r*6+c]=s;
    }
    // BPA = B^T * PA  (3x6)
    double BPA[18];
    for(int r=0;r<3;r++) for(int c=0;c<6;c++){
        double s=0.0; for(int k=0;k<6;k++) s+=BT[r*6+k]*PA[k*6+c];
        BPA[r*6+c]=s;
    }
    // M = Rd + B^T P B   (nochmal mit finalem P)
    for(int r=0;r<6;r++) for(int c=0;c<3;c++){
        double s=0.0; for(int k=0;k<6;k++) s+=P[r*6+k]*Bd_d[k*3+c];
        PB[r*3+c]=s;
    }
    for(int r=0;r<3;r++) for(int c=0;c<3;c++){
        double s=0.0; for(int k=0;k<6;k++) s+=BT[r*6+k]*PB[k*3+c];
        BT_P_B[r*3+c]=s;
    }
    for(int i=0;i<9;i++) M[i]=Rd_d[i]+BT_P_B[i];
    if(!chol3_d(M,L)) return false;

    // K = solve(M, BPA)
    double Kd[18];
    for(int col=0; col<6; ++col){
        double b[3]={ BPA[0*6+col], BPA[1*6+col], BPA[2*6+col] };
        double x[3]; chol3_solve_d(L,b,x);
        Kd[0*6+col]=x[0]; Kd[1*6+col]=x[1]; Kd[2*6+col]=x[2];
    }

    // Outputs
    for(int i=0;i<18;i++) K[i]=(float)Kd[i];
    if(Pout) for(int i=0;i<36;i++) Pout[i]=(float)P[i];
    return true;
}

static void lqr_axis_2x2(double Ts, double qp, double qv, double rr,
                         double tol, int max_it,
                         double *Kp, double *Kv, double *P11, double *P12, double *P22)
{
    const double Ts2 = Ts*Ts;
    const double b1  = 0.5*Ts2;
    const double b2  = Ts;

    double p11 = qp, p12 = 0.0, p22 = qv;

    for (int it = 0; it < max_it; ++it) {
        // M = R + B^T P B = rr + 0.25*p11*Ts^4 + p12*Ts^3 + p22*Ts^2
        double M = rr + 0.25*p11*Ts2*Ts2 + p12*Ts*Ts2 + p22*Ts2;
        if (!(M > 0.0)) M = rr + 1e-12;

        // S = A^T P B, mit A = [1 Ts; 0 1], B = [b1; b2]
        double pb1 = p11*b1 + p12*b2;
        double pb2 = p12*b1 + p22*b2;
        double S1  = pb1;
        double S2  = Ts*pb1 + pb2;

        // A^T P A (geschlossen)
        double ATPA11 = p11;
        double ATPA12 = p11*Ts + p12;
        double ATPA22 = p11*Ts2 + 2.0*p12*Ts + p22;

        // Riccati-Update: Pnext = A^T P A - S S^T / M + Q
        double invM = 1.0 / M;
        double n11 = ATPA11 - S1*S1*invM + qp;
        double n12 = ATPA12 - S1*S2*invM + 0.0;
        double n22 = ATPA22 - S2*S2*invM + qv;

        double d = fmax(fabs(n11 - p11), fmax(fabs(n12 - p12), fabs(n22 - p22)));
        p11 = n11; p12 = n12; p22 = n22;
        if (d < tol) break;
    }

    // K = (R + B^T P B)^-1 * (B^T P A)
//    double Ts2 = Ts*Ts;
    double M   = rr + 0.25*p11*Ts2*Ts2 + p12*Ts*Ts2 + p22*Ts2;
    if (!(M > 0.0)) M = rr + 1e-12;

    // B^T P A = [ b1 b2 ] * (P A),  P A = [[p11, p11*Ts + p12],[p12, p12*Ts + p22]]
    double bpa1 = 0.5*p11*Ts2 + p12*Ts;                      // Gain auf Position
    double bpa2 = 0.5*p11*Ts2*Ts + 1.5*p12*Ts2 + p22*Ts;     // Gain auf Geschwindigkeit

    *Kp = bpa1 / M;
    *Kv = bpa2 / M;
    if (P11) *P11 = p11;
    if (P12) *P12 = p12;
    if (P22) *P22 = p22;
}

bool lqr_dare_iter2x2(const float Ad[36], const float Bd[18],
                      const float Qd[36], const float Rd[9],
                      float K[18], float Pout[36],
                      float tol, int max_it)
{
    // Ts aus Ad
    double Ts = (double)Ad[0*6 + 3];
    if (!(Ts > 0.0)) return false;

    // Arrays nullen
    memset(K, 0, 18*sizeof(float));
    if (Pout) memset(Pout, 0, 36*sizeof(float));

    // Für jede Achse unabhängig (Position i, Geschwindigkeit i+3)
    for (int axis = 0; axis < 3; ++axis) {
        int pos = axis, vel = axis + 3;

        // Diagonale Gewichte (diskret! -> Qd = Q*Ts, Rd = R*Ts)
        double qp = (double)Qd[pos*6 + pos];
        double qv = (double)Qd[vel*6 + vel];
        double rr = (double)Rd[axis*3 + axis];

        // *** HARTES TOL/IT, damit auch die z-Achse exakt wird ***
        double Kp, Kv, P11, P12, P22;
        double tol_d   = (double)(tol > 0.0f ? tol : 1e-18);
        int    it_dmax = (max_it > 0 ? max_it : 200000);

        lqr_axis_2x2(Ts, qp, qv, rr, tol_d, it_dmax, &Kp, &Kv, &P11, &P12, &P22);

        K[axis*6 + pos] = (float)Kp;
        K[axis*6 + vel] = (float)Kv;

        if (Pout) {
            Pout[pos*6 + pos] = (float)P11;
            Pout[pos*6 + vel] = (float)P12;
            Pout[vel*6 + pos] = (float)P12;
            Pout[vel*6 + vel] = (float)P22;
        }
    }
    return true;
}

// Prüft Residuum der diskreten DARE für 2x2 Doppelintegrator
// Gibt den Maximalfehler zurück
double lqr_residuum_axis(double Ts,
                                double qp, double qv, double rr,
                                double p11, double p12, double p22)
{
    const double Ts2 = Ts*Ts;
    const double b1  = 0.5*Ts2;
    const double b2  = Ts;

    // A, B
    double A11=1.0, A12=Ts, A21=0.0, A22=1.0;
    double B1=b1, B2=b2;

    // P-Matrix
    double P[2][2] = { {p11, p12}, {p12, p22} };

    // Q, R
    double Q[2][2] = { {qp, 0.0}, {0.0, qv} };
    double R = rr;

    // R + B^T P B
    double BtPB = B1*(p11*B1 + p12*B2) + B2*(p12*B1 + p22*B2);
    double M = R + BtPB;
    if (!(M>0.0)) M = R + 1e-12;

    // A^T P A
    double PA11 = p11*A11 + p12*A21;
    double PA12 = p11*A12 + p12*A22;
    double PA21 = p12*A11 + p22*A21;
    double PA22 = p12*A12 + p22*A22;
    double ATPA11 = A11*PA11 + A21*PA21;
    double ATPA12 = A11*PA12 + A21*PA22;
    double ATPA21 = A12*PA11 + A22*PA21;
    double ATPA22 = A12*PA12 + A22*PA22;

    // A^T P B
    double S1 = A11*(p11*B1+p12*B2) + A21*(p12*B1+p22*B2);
    double S2 = A12*(p11*B1+p12*B2) + A22*(p12*B1+p22*B2);

    // S * M^-1 * S^T
    double invM = 1.0/M;
    double SM11 = S1*S1*invM;
    double SM12 = S1*S2*invM;
    double SM22 = S2*S2*invM;

    // rechte Seite der DARE
    double RHS11 = ATPA11 - SM11 + qp;
    double RHS12 = ATPA12 - SM12 + 0.0;
    double RHS22 = ATPA22 - SM22 + qv;

    // Differenzmatrix (Residuum)
    double d11 = p11 - RHS11;
    double d12 = p12 - RHS12;
    double d22 = p22 - RHS22;

    // Maximum Norm
    double maxerr = fmax(fabs(d11), fmax(fabs(d12), fabs(d22)));
    return maxerr;
}

// Exakter Stabilitätscheck für 3x (2x2) Doppel-Integrator
// Eingabe:  Ad (6x6, row-major), K (3x6, row-major)
// Ausgabe:  *rho_max  = max |lambda| über alle Achsen
//           rho_axis[3] = |lambda|max je Achse
//           lam1/lam2   = Eigenwerte je Achse (reell; bei komplex: conj-Paar -> gleiche |.|)
// Rückgabe: true, wenn stabil (rho_max < 1), sonst false.

bool lqr_stability_check_2x2(const float Ad[36], const float K[18],
                                    float *rho_max,
                                    float rho_axis[3],
                                    float lam1[3], float lam2[3])
{
    // Ts aus Ad: Ad(0,3) = Ts (für Doppelintegrator mit ZOH)
    const float Ts = Ad[0*6 + 3];
    if (!(Ts > 0.0f)) return false;
    const float Ts2 = Ts*Ts;

    float rhoMax = 0.0f;

    for (int axis = 0; axis < 3; ++axis) {
        // Achsen-Gains: K-Zeile = axis, Spalten pos=axis, vel=axis+3
        const float Kp = K[axis*6 + axis];
        const float Kv = K[axis*6 + (axis+3)];

        // A_cl (2x2) = [1 Ts; 0 1] - [0.5*Ts^2; Ts] * [Kp Kv]
        const float a11 = 1.0f - 0.5f*Ts2*Kp;
        const float a12 = Ts   - 0.5f*Ts2*Kv;
        const float a21 =      - Ts * Kp;
        const float a22 = 1.0f - Ts * Kv;

        // Eigenwerte eines 2x2: lambda = (tr ± sqrt(tr^2 - 4*det))/2
        const float tr  = a11 + a22;
        const float det = a11*a22 - a12*a21;
        const float disc = tr*tr - 4.0f*det;

        float ev1_abs, ev2_abs, ev1, ev2;

        if (disc >= 0.0f) {
            // reelle Eigenwerte
            const float s = sqrtf(disc);
            ev1 = 0.5f*(tr + s);
            ev2 = 0.5f*(tr - s);
            ev1_abs = fabsf(ev1);
            ev2_abs = fabsf(ev2);
        } else {
            // komplex-konjugiertes Paar: |lambda| = sqrt(det)
            const float r = (det >= 0.0f) ? sqrtf(det) : 0.0f; // numerische Vorsicht
            ev1 = ev2 = 0.0f; // reeller Anteil uninteressant hier
            ev1_abs = ev2_abs = r;
        }

        const float rho = (ev1_abs > ev2_abs) ? ev1_abs : ev2_abs;
        rho_axis[axis] = rho;
        lam1[axis] = ev1; lam2[axis] = ev2;
        if (rho > rhoMax) rhoMax = rho;
    }

    if (rho_max) *rho_max = rhoMax;
    return (rhoMax < 1.0f);
}


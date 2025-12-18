import numpy as np
from matplotlib import pyplot as plt

G = 6.674e-11
M_jord = 5.972e24
R_jord = 6371000
H = 7400  # Scale height
u = 3000  # Udstødningshastighed


m_payload = 100
m_dry = 26200
m_start = 525000 + m_payload


dm_rate_1 = 2100  
dm_rate_2 = 2100  

x0 = 0
n = 2500 
delta_t = 0.1
β = 1.957

t = np.zeros(n)


x = np.zeros(n);
v = np.zeros(n);
m = np.zeros(n)
x_p = np.zeros(n);
v_p = np.zeros(n);
m_p = np.zeros(n)

# Startværdier
m[0] = m_start
x[0] = 0.1 
v[0] = 0

m_p[0] = m_start
x_p[0] = 0.01
v_p[0] = 0

for i in range(1, n):
    t[i] = t[i - 1] + delta_t


    if m[i - 1] > m_dry:
        dm = dm_rate_1
        if m[i - 1] - (dm * delta_t) < m_dry:
            dm = (m[i - 1] - m_dry) / delta_t  # Tøm resten

        løft = u * dm  # Thrust = u * dm/dt (Bemærk: dm her er raten kg/s)

    else:
        dm = 0
        løft = 0


    m[i] = m[i - 1] - dm * delta_t

    tyngde = (G * M_jord) / (R_jord + x[i - 1]) ** 2


    luft = (β * np.exp(-x[i - 1] / H)) * (v[i - 1] * abs(v[i - 1])) / m[i - 1]


    acc_motor = løft / m[i - 1]
    acc_tyngde = tyngde  

    a = acc_motor - acc_tyngde - luft

    # Euler skridt
    v[i] = v[i - 1] + a * delta_t
    x[i] = x[i - 1] + v[i] * delta_t

    # Gulv-tjek
    if x[i] <= 0:
        x[i] = 0
        v[i] = 0


    if m_p[i - 1] > m_dry:
        dm_p = dm_rate_2
        if m_p[i - 1] - (dm_p * delta_t) < m_dry:
            dm_p = (m_p[i - 1] - m_dry) / delta_t
        løft_p = u * dm_p
    else:
        dm_p = 0
        løft_p = 0

    m_p[i] = m_p[i - 1] - dm_p * delta_t

    tyngde_p = (G * M_jord) / (R_jord + x_p[i - 1]) ** 2
    luft_p = (β * np.exp(-x_p[i - 1] / H)) * (v_p[i - 1] * abs(v_p[i - 1])) / m_p[i - 1]

    acc_motor_p = løft_p / m_p[i - 1]

    a_p = acc_motor_p - tyngde_p - luft_p

    v_p[i] = v_p[i - 1] + a_p * delta_t
    x_p[i] = x_p[i - 1] + v_p[i] * delta_t

    print (x[i],v[i])
    #print (x_p[i],v_p[i])
    if x_p[i] <= 0:
        x_p[i] = 0
        v_p[i] = 0


plt.figure(figsize=(10, 8))

plt.subplot(2, 1, 1)
plt.plot(t, x, label=f'Raket 1 (dm={dm_rate_1} kg/s)')
#plt.plot(t, x_p, label=f'Raket 2 (dm={dm_rate_2} kg/s)')
plt.ylabel('Højde [m]')
plt.title('Raketopsendelse: Sammenligning af massestrøm')
plt.grid(True)
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(t, v, color = 'green', label='Hastighed 1')
#plt.plot(t, v_p, label='Hastighed 2')
plt.xlabel('Tid [s]')
plt.ylabel('Hastighed [m/s]')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()

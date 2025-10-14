import numpy as np
import roboticstoolbox as rtb
import spatialmath as sm
import matplotlib.pyplot as plt
from roboticstoolbox import DHRobot
from scipy.linalg import null_space
from roboticstoolbox.tools.trajectory import ctraj, jtraj


# Extiendo la clase DHRobot para que incluya el generador de trayectorias joint y cartesiano
class DHRobotGT(DHRobot):
    # Defino algunas variables de la clase, que tienen sentido cuando se generan trayectorias
    t_ref=[]; q_ref=[]; qd_ref=[]; qdd_ref=[]; tau=[];

    def get_control_reference(self,t):
        if len(self.t_ref) == 0:
            raise ValueError("No hay trayectorias deseadas definidas")

        # busca el índice dentro del vector
        k = np.searchsorted(self.t_ref, t, side='right') - 1

        # Dejo al índice dentro del rango
        if k < 0:
            k = 0
        elif k >= len(self.t_ref):
            k = len(self.t_ref) - 1

        return self.t_ref[k],self.q_ref[k],self.qd_ref[k],self.qdd_ref[k]

    def __init__(self, *args, tacc=0.1, Ts=1E-3,vmax=np.radians(150)*np.ones(6), **kwargs):
        super().__init__(*args, **kwargs)
        self.tacc = tacc
        self.Ts = Ts
        self.vmax = vmax

    def interpoladorTrapezoidal(self,A,B,C,Tj, return_s=False, plot=False):
        """
        Interpolador trapezoidal en zona 1 y 2

        Args
        ----
          A: punto en el que estoy
          B: punto al que estaba yendo en el segmento anterior
          C: punto al que voy
          Tj: tiempo en que se realiza el movimiento

        Returns
        --------
          q_aux: vector interpolado de posiciones
          qd_aux: vector interpolado de velocidades
          qdd_aux: vector interpolado de aceleraciones
        """
        DA = A-B
        DC = C-B

        # Zona 1
        # Generar el vector tseg para Zona 1
        tseg = np.arange(-self.tacc + self.Ts, self.tacc + self.Ts, self.Ts)

        # Calculo las referencias para zona 1
        qdd_aux = np.outer((DC/Tj+DA/self.tacc)/(2*self.tacc),np.ones(len(tseg)))
        qd_aux = (DC / Tj)[:, np.newaxis] * (tseg + self.tacc) / (2 * self.tacc) + (DA / self.tacc)[:, np.newaxis] * (tseg - self.tacc) / (2 * self.tacc)
        q_aux = (DC / Tj)[:, np.newaxis] * (tseg + self.tacc)**2 / (4 * self.tacc) + (DA / self.tacc)[:, np.newaxis] * (tseg - self.tacc)**2 / (4 * self.tacc) + np.outer(B,np.ones(len(tseg)))

        # Zona 2
        # Generar el vector tseg para Zona 2
        tseg = np.arange(self.tacc + self.Ts, Tj - self.tacc + 0.5* self.Ts , self.Ts)

        # Inicializar las matrices theta2p, thetap y theta
        qdd_aux = np.hstack([qdd_aux,np.zeros((len(B), len(tseg)))])   # Suponiendo que B es un vector
        qd_aux = np.hstack([qd_aux,np.outer(DC / Tj, np.ones(len(tseg)))])
        q_aux = np.hstack([q_aux,np.outer(DC / Tj, tseg) +  np.outer(B,np.ones(len(tseg)))])
        # print(f'q_aux\n{q_aux}')
        # print(f'q_aux[-1] - q_aux[0]\n{q_aux[-1] - q_aux[0]}')
        # print(f'q_aux[-1] = {q_aux[-1]}\nq_aux[0,0]={q_aux[0]}')
        # print(f'q_aux shape = {q_aux.shape}')
        # Diferencia total en cada coordenada
        deltas = q_aux[:,-1] - q_aux[:,0]
        # print(f'Deltas\n{deltas}')

        # Busco el índice del eje con mayor desplazamiento absoluto
        idx = np.argmax(np.abs(deltas))

        # Normalizo con respecto a ese eje
        delta = deltas[idx]
        if np.isclose(delta, 0.0):
            print("No hay ninguna coordenada con desplazamiento para normalizar.")
        else:
            q_norm = (q_aux[idx,:] - q_aux[idx,0]) / delta

        qd_max = np.max(np.abs(qd_aux[idx,:]))
        qdd_max = np.max(np.abs(qdd_aux[idx,:]))

        qd_norm = qd_aux[idx,:] / qd_max if qd_max > 0 else np.zeros_like(qd_aux[idx,:])
        qdd_norm = qdd_aux[idx,:] / qdd_max if qdd_max > 0 else np.zeros_like(qdd_aux[idx,:])

        # q_norm = (q_aux[2,:] - q_aux[2,0]) / (q_aux[2,-1] - q_aux[2,0])
        # qd_norm = qd_aux[0,:] / np.max(np.abs(qd_aux[0,:]))
        # qdd_norm = qdd_aux[0,:] / np.max(np.abs(qdd_aux[0,:]))
        s_profile = q_norm 

        if plot:
            import matplotlib.pyplot as plt
            plt.figure()
            plt.plot(q_norm, label='q (posición)', linewidth=2)
            plt.plot(qd_norm, label='qd (velocidad)', linewidth=2)
            plt.plot(qdd_norm, label='qdd (aceleración)', linewidth=2)
            plt.xlabel('Step')
            plt.ylabel('Normalizado')
            plt.legend()
            plt.grid(True)
            plt.title('Perfiles trapezoidales normalizados')
            plt.show()

        if return_s:
            return q_aux, qd_aux, qdd_aux, s_profile
        else:
            return q_aux, qd_aux, qdd_aux

    def genTrJoint(self, q_dest,Td):
        """
        Genera la trayectoria joint para un conjunto de puntos de paso

        Args
        -----
          q_dest : Matriz con los puntos de paso. Cada fila corresponde a un punto
          Td : tiempos deseados de cada movimiento

        Returns
        -------
          t_ref : Vector de tiempo de referencia
          q_ref : Vector de posiciones articulares de referencia
          qd_ref : Vector de velocidades articulares de referencia
          qdd_ref : Vector de aceleraciones articulares de referencia
          POSES : Vector de posiciones cartesianas de referencia
        """
        q = np.empty((self.nlinks,0)); qd = np.empty((self.nlinks,0)); qdd = np.empty((self.nlinks,0))
        A = q_dest[0,:];
        for i in range(len(q_dest)-1):
          B = q_dest[i,:]
          if i<len(q_dest)-1:
            C = q_dest[i+1,:]
          else:
            C = B
            Td[i] = 0
          Tj = np.max((np.max(np.abs(C-B)/self.vmax),Td[i],2*self.tacc))
          q_aux,qd_aux,qdd_aux = self.interpoladorTrapezoidal(A,B,C,Tj)
          q = np.hstack([q,q_aux]); qd = np.hstack([qd,qd_aux]); qdd = np.hstack([qdd,qdd_aux]);
          A = q[:,-1]
        t = np.linspace(0, q.shape[1],num=q.shape[1])*self.Ts
        # print(f'q0: {np.rad2deg(q[::6])}')
        # Calculo la trayectoria cartesiana deseada
        POSES = self.fkine(q.transpose()) # .extend([self.fkine(q[:,i]) for i in range(q.shape[1])])
        self.t_ref = t; self.q_ref=q.T; self.qd_ref=qd.T; self.qdd_ref=qdd.T
        self.tau = np.zeros_like(self.q_ref)
        return self.t_ref, self.q_ref, self.qd_ref, self.qdd_ref

    def genTrCart(self,POSE_dest,Td, conf = [1, 1, 1]):
    
        """
        Genera la trayectoria cartesiana para un conjunto de puntos de paso

        Args
        ----
          POSE_dest : Lista con las POSES de paso
          Td : tiempos deseados de cada movimiento

        Returns
        -------
          t_ref : Vector de tiempo de referencia
          q_ref : Vector de posiciones articulares de referencia
          qd_ref : Vector de velocidades articulares de referencia
          qdd_ref : Vector de aceleraciones articulares de referencia
          POSES : Vector de posiciones cartesianas de referencia
        """
        n_tramos = len(POSE_dest)-1
        POSEA = POSE_dest[0]
        POSES = []
        s_profiles = []
        for i in range(n_tramos):
            # print(f'Analizando la pose de índice {i} de {len(POSE_dest)}')
            # print(f'POSEA\n{POSEA}')
            POSEB = POSE_dest[i]
            if i<len(POSE_dest)-1:
                POSEC = POSE_dest[i+1]
            else:
                POSEC = POSEB
                Td[i] = 0
            # print(f'POSEB\n{POSEB}')
            # print(f'POSEC\n{POSEC}')
            Tj = np.max([Td[i],2*self.tacc])
            _, _, _, s_profile = self.interpoladorTrapezoidal(POSEA.t, POSEB.t, POSEC.t, Tj, return_s=True)
            # print(f'Forma de s_profile: {s_profile.shape}')
            s_profiles.append(s_profile/n_tramos + i / n_tramos)
            # print(f's_profile {i} \n{s_profiles[i]}')
        s_profilefull = np.concatenate(s_profiles)
        # print(f's_profiles completo\n{s_profilefull}')
        q = np.zeros((len(POSES),self.nlinks))
        for i in range(len(POSES)):
          q[i,:],_ = self.ikine(POSES[i], conf)

        POSES = POSEA.interp(POSEB, s=s_profilefull.round(3))
        # import matplotlib.pyplot as plt
        # plt.figure()
        # plt.plot(s_profilefull.round(4), linewidth=2)
        # plt.xlabel('Step')
        # plt.ylabel('s')
        # plt.legend()
        # plt.grid(True)
        # plt.title('Perfiles de SLERP')
        # plt.show()
        # for pose_calc in range(100):
        #     print(f'[DEBUG] Pose calculada {pose_calc}:\n{POSES[pose_calc]}')

        q = np.zeros((len(POSES), self.nlinks))
        for i in range(len(POSES)):
            # print(f'La última pose calculada fue la {i}')
            # print(f'Ult pose calculada\n{POSES[i]}')
            q[i,:], _ = self.ikine(POSES[i], conf)

        qd = np.diff(q, axis=0) / self.Ts
        qd = np.vstack([qd, np.zeros(self.nlinks,)])
        qdd = np.diff(qd, axis=0) / self.Ts
        qdd = np.vstack([qdd, np.zeros(self.nlinks,)])

        t = np.linspace(0, len(q), num=len(q)) * self.Ts
        self.t_ref = t; self.q_ref = q; self.qd_ref = qd; self.qdd_ref = qdd
        self.tau = np.zeros_like(self.q_ref)
        return self.t_ref, self.q_ref, self.qd_ref, self.qdd_ref
    
class myCobot320(DHRobotGT):
    def __init__(self,*args, metros=False, rotar_base=False, **kwargs):
        self.rotar_base = rotar_base
        # Definición de los enlaces usando parámetros DH. Por defecto en mm
        esc = 1e-3 if metros else 1
        
        eje1 = rtb.RevoluteDH(alpha=-np.pi/2,a=0,d=173.87*esc,offset=0,qlim=[-168*np.pi/180,168*np.pi/180])
        eje2 = rtb.RevoluteDH(alpha=0,a=134.96*esc,d=0,offset=-np.pi/2,qlim=[-135*np.pi/180,135*np.pi/180])
        eje3 = rtb.RevoluteDH(alpha=0,a=119.99*esc,d=0,offset=0,qlim=[-150*np.pi/180,150*np.pi/180]) 
        eje4 = rtb.RevoluteDH(alpha=np.pi/2,a=0,d=88.74*esc,offset=np.pi/2,qlim=[-145*np.pi/180,145*np.pi/180])
        eje5 = rtb.RevoluteDH(alpha=-np.pi/2,a=0,d=94.99*esc,offset=0,qlim=[-165*np.pi/180,165*np.pi/180])    
        eje6 = rtb.RevoluteDH(alpha=0,a=0,d=65.48*esc,offset=0,qlim=[-180*np.pi/180,180*np.pi/180])
    
        # Crear la estructura del robot
        super().__init__(*args,[eje1, eje2, eje3, eje4, eje5, eje6], name='myCobot320',gravity = np.array([0, 0, -9.8]),**kwargs)
        
        # Rotar la base para que coincida con pyMyCobot
        if rotar_base:
            self.base = sm.SE3.Rz(np.pi)

    def ikine(self, POSE, conf=np.array([1,1,1]), offset = True, graficar_alcance = False):
        """
        Cinemática Inversa del robot myCobot320

        Parameters
        ----------
        POSE : pose de destino. POSE.R corresponde a la rotación (3x3) y POSE.t a la traslación (3x1)
        conf : vector de configuraciones (brazo, codo, muñeca)

        Returns
        -------
        q : vector de variables articulares
        """
        conf1, conf2, conf3 = conf

        if len(POSE) == 7:   ### Esto lo hago para permitir al metodo funcionar con una pose extraida de fkine y de fkine_all
            pose_aux = POSE[6]
        elif self.rotar_base:
            pose_aux = sm.SE3.Rz(np.pi) * POSE
        else:
            pose_aux = POSE

        # Extraigo las componentes de la matriz que van a ser usadas en las ecuaciones
        px, py, pz = pose_aux.t
        nx, ny, nz = pose_aux.R[:, 0]   
        sx, sy, sz = pose_aux.R[:, 1]
        ax, ay, az = pose_aux.R[:, 2]  
        # # Adecuo las variables POSE 
        # px, py, pz = POSE.t
        # nx, ny, nz = POSE.R[:,0]
        # sx, sy, sz = POSE.R[:,1]
        # ax, ay, az = POSE.R[:,2]

        # Tomo los largos de eslabones de la tabla DH
        d1 = self.links[0].d
        a2 = self.links[1].a
        a3 = self.links[2].a
        d4 = self.links[3].d 
        d5 = self.links[4].d 
        d6 = self.links[5].d 

        # Theta1: indeterminación del brazo
        discr = (px - ax*d6)**2 + (py - ay*d6)**2 - d4**2
        if discr<0:
            raise IKineError(
            "Discriminante negativo. La pose está fuera del círculo de alcance."
            )
            # return [],-1
        q1 = np.arctan2(d4, conf1*np.sqrt(discr)) - np.arctan2(py - ay*d6, ax*d6 - px)
        # q1 = -2*np.arctan((-ax*d6 + px + conf1*np.sqrt(discr))/(-ay*d6 + d4 + py))

        # Theta5: indeterminación de la muñeca
        q5 = np.arctan2(conf3*np.sqrt((ny*np.cos(q1) - nx*np.sin(q1))**2 + (sy*np.cos(q1) - sx*np.sin(q1))**2), ay*np.cos(q1) - ax*np.sin(q1))
        
        # Theta6: singularidad si sin(q5)=0
        q6 = np.arctan2(-conf3*(sy*np.cos(q1) - sx*np.sin(q1)), conf3*(ny*np.cos(q1) - nx*np.sin(q1)))
        # q6 = np.arctan2((sx*np.sin(q1) - sy*np.cos(q1))/np.sin(q5), (ny*np.cos(q1) - nx*np.sin(q1))/np.sin(q5))
        
        # Calculo theta2+theta3+theta4
        theta234 = np.arctan2(az*conf3, -conf3*(ax*np.cos(q1) + ay*np.sin(q1)))
        # theta234 = np.arctan2(az/np.sin(q5), -(ax*np.cos(q1) + ay*np.sin(q1))/np.sin(q5))

        # Cálculos auxiliares: sustituciones A y B
        A = px*np.cos(q1) - d5*np.sin(theta234) + py*np.sin(q1) + d6*np.sin(q5)*np.cos(theta234)
        B = d1 - pz + d5*np.cos(theta234) + d6*np.sin(q5)*np.sin(theta234)
        # Calculo c3 
        c3 = (A**2 + B**2 - a2**2 - a3**2)/(2*a2*a3)
        # Chequeo que el punto sea alcanzable
        if np.abs(c3)>1:
            raise IKineError(
            "Alcanzabilidad. c3 fuera de [-1,1], punto no alcanzable."
        )
            # return [],-1
        # Theta3: indeterminación del codo 
        q3 = np.arctan2(conf2*np.sqrt(1 - c3**2), c3)
        
        # Calculo el seno y el coseno de theta2
        s2 = (B*a2 + B*a3*np.cos(q3) - A*a3*np.sin(q3))/(a2**2 + a3**2 + 2*a2*a3*np.cos(q3))
        c2 = (A*a2 + A*a3*np.cos(q3) + B*a3*np.sin(q3))/(a2**2 + a3**2 + 2*a2*a3*np.cos(q3))
        # Calculo theta2
        q2 = np.arctan2(s2, c2)
        
        # Calculo theta4
        q4 = theta234 - q2 - q3
        
        q = np.array([q1,q2,q3,q4,q5,q6]) #- self.offset
        if offset:
            q = q - self.offset
        # Limito q entre -pi y pi
        q = (q + np.pi) % (2 * np.pi) - np.pi
        status=1

        if graficar_alcance:
            r_min = abs(a2 - a3)
            r_max = a2 + a3

            posiciones = [np.array([0, 0, 0])]
        
            for i in range(7):
                posiciones.append(POSE.A[i][:3, 3])  # Posición X, Y, Z

            pos=np.array(posiciones).T
            fig = plt.figure(figsize=(10,10))
            ax3d = fig.add_subplot(111, projection='3d')

            # Posición inicial
            ax3d.plot(pos[0], pos[1], pos[2], 'o', color='red', label='Posición', zorder=3, markersize=8)
            # ejes = ['1', '2', '3', '4', '5', '6']

            # for i in range(len(pos[0])-2):  # Iterate through consecutive points
            #     mid_x = (pos[0, i+1] + pos[0, i+2]) / 2
            #     mid_y = (pos[1, i+1] + pos[1, i+2]) / 2
            #     mid_z = (pos[2, i+1] + pos[2, i+2]) / 2 
            #     ax3d.text(mid_x, mid_y, mid_z, ejes[i], color='black', fontsize=9, zorder=3,
            #               bbox=dict(facecolor='white', edgecolor='black'))

            # Uno los puntos intermedios para visualizar los eslabones
            for i in range(len(pos[0]) - 1):
                ax3d.plot(
                    [pos[0, i], pos[0, i + 1]],
                    [pos[1, i], pos[1, i + 1]],
                    [pos[2, i], pos[2, i + 1]],
                    color='black', linewidth=3
                )
            muñeca = np.array([px - ax * d6,
                    py - ay * d6,
                    pz - az * d6])
            theta = np.linspace(0, 2 * np.pi, 100)
            x_circ = d4 * np.cos(theta)
            y_circ = d4 * np.sin(theta)
            centro_z = muñeca[2] 
            z_circ = np.full_like(theta, centro_z)

            x_max = r_max * np.cos(theta)
            y_max = r_max * np.sin(theta)

            x_min = r_min * np.cos(theta)
            y_min = r_min * np.sin(theta)

            ax3d.plot(x_circ, y_circ, z_circ,
                    color='blue',
                    linestyle='-.',
                    linewidth=2,
                    label='Límite de alcance (discriminante)')
            ax3d.plot(x_max, y_max, d1, 'b--', linewidth=1, label="Alcance Máximo (c3)")
            ax3d.plot(x_min, y_min, d1,  'g--', linewidth=1, label="Alcance Mínimo (c3)")

            # Configuración del gráfico
            ax3d.set_xlim([-0.300, 0.300])
            ax3d.set_ylim([-0.300, 0.300])
            ax3d.set_zlim([0.000, 0.400])
            ax3d.set_xlabel('X (mm)', fontsize=10)
            ax3d.set_ylabel('Y (mm)', fontsize=10)
            ax3d.set_zlabel('Z (mm)', fontsize=10)
            plt.title('myCobot320: Posición de los Eslabones', fontsize=16)
            plt.legend(fontsize=11)
            plt.grid(True)
            plt.show()
            
        return q,status
    
    def calc_conf(self,q):
        
        """
        Funcion que calcula la configuracion a partir de las variables articulares
            - p16_x: es la componente x de la transformacion que expresa la pos y orientacion del
            efector final en el sist de ref del primer eslabon. Su signo indica si el hombro
            esta hacia adelante o hacia atras.
            - El angulo de q3 me da conf2, que es el codo arriba o abajo
            - El angulo de q5 me da conf3, que es la muñeca arriba o abajo
        """
        redimensionar = False
        if q.ndim == 1:
            redimensionar = True
            q = q[np.newaxis,:]
        result = []
        for q_row in q:
            A = self.fkine_all(q_row)
            p16_x = (np.linalg.inv(A[1]) @ A[6].A)[0, -1]
            result.append([-np.sign(p16_x), np.sign(q_row[2]), np.sign(q_row[4])])
        conf = np.array(result, dtype=int)
        if redimensionar:
            conf=conf.flatten()
        return conf
    
    def graficar_conf(self, q, conf, limits=np.array([-0.100, 0.300, -0.300, 0.300, 0.000, 0.400])):
    
        """
        Funcion que grafica una configuracion dada que recibe como argumento.
        Se captura el entorno generado por la funcion plot de la libreria para modificarle los ejes
        Tambien se guarda como png con un nombre que permite identificarla por configuracion. 
        
        """
        # conf = self.calc_conf(q)
        a = self.plot(q, backend='pyplot', limits= limits  , jointaxes = False ,block=False, name=False)
        plt.close()
        fig = a.fig
        fig.set_size_inches(6,6)
        ax = fig.gca()
        ax.view_init(elev=41, azim=-37)
        ax.set_xlim([limits[0], limits[1]])
        ax.set_ylim([limits[2], limits[3]])
        ax.set_zlim([limits[4], limits[5]])
        ax.set_box_aspect([1, 1, 0.5]) 
        # a.hold()
        a.close()
        fig.savefig(f'Imágenes\Configuraciones\\[{conf[0]}][{conf[1]}][{conf[2]}].png')
        return fig
    
    def traj_nucleo(self, q, puntos, paso, norm):
        """
        Funcion que genera una trayectoria en la direccion del nucleo del jacobiano. 
        Recibe como parametro un vector de variables articulares, la cantidad de puntos
        de la trayectoria, el paso con el que se debe avanzar y el parametro de normalizacion.
        """
        trayectoria_total = []
        q_actual = q

        for i in range(puntos):
            J_actual = self.jacob0(q_actual)
            ns = null_space(J_actual)
            if ns.size == 0:                                  # Valido si sigue existiendo el nucleo
                print(f"En la iteracion {i} el jacobiano recupera rango. Es decir: la singularidad es externa!")
                return 0
            direccion = ns[:, 0]
            direccion = direccion / direccion[norm]*-1        # Normalizo para que la direccion de la trayectoria sea consistente
            q_siguiente = q_actual + paso * direccion         # Avanzo un paso en la direccion del nucleo del jacobiano
            #traj_segmento = jtraj(q_actual, q_siguiente, 2).q # Construyo una trajectoria joint ejtre los dos q
            tr_segmento = self.genTrJoint(np.array([q_actual, q_siguiente, q_siguiente]), 0*np.ones(3))
            traj_segmento = self.q_ref[::300]
            trayectoria_total.append(traj_segmento[1:])       # Acumulo la trayectoria total
            q_actual = q_siguiente                            # Actualizo el q actual
        print("Trayectoria generada con exito. La singularidad es interna!")
        return np.vstack(trayectoria_total)

class IKineError(Exception):
    """Excepción lanzada por ikine cuando no se puede calcular la solución."""
    def __init__(self, mensaje: str):
        super().__init__(mensaje)

    def __repr__(self):
        return f"IKineError(error={self.args[0]})"

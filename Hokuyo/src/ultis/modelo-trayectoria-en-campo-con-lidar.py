import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
import pandas as pd

def Impresion_puntos(df, lidar_fixed_ang, width, length):
    # Imprimir en pantalla
    fig,ax = plt.subplots()
    ax.set_title("3d meassure plane",fontsize=14,fontweight="bold")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    if(width != 0) or (length != 0):
        ax.set_xlim(0, width)
        ax.set_ylim(0, length)
    for ring in range(0,len(lidar_fixed_ang)):                                        # Impresión de los puntos con colores para identificar a que ángulo de lidar hace referencia
        positions = np.where(df[:,3]==ring)
        leyenda = "rings=" + str(lidar_fixed_ang[ring])
        ax.plot(df[positions,0].T,df[positions,1].T,".",label=leyenda)
    plt.legend()
    plt.tight_layout()
    plt.show()

def Impresion_recorrido(df_p, df_r, width, length):
    # Imprimir en pantalla
    fig,ax = plt.subplots()
    ax.set_title("3d meassure plane",fontsize=14,fontweight="bold")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_xlim(0, width)
    ax.set_ylim(0, length)
    ax.plot(df_p[:,0].T,df_p[:,1].T,"x")
    ax.plot(df_r[:,0].T,df_r[:,1].T)
    plt.tight_layout()
    plt.show()

def Proyeccion_angular_lidar(alpha,tX,tY,tZ):
    # Variables--------------------------------------
    alpha = np.radians(alpha)                                                         # Ángulo de inclinación del pilotScan
    lidar_fixed_ang = np.array([-17,-14.17,-11.33,-8.50,-5.67,-2.83,0,3])             # Ángulo del lidar
    gra_phi = 90 - lidar_fixed_ang                                                    # Traslación del ángulo del lidar al modelo matemático
    phi = np.radians(gra_phi)                                                         # Transformación de grados a radianes
    angular_ring_resolution = 0.06                                                  # Resolucion por anillo
    n_data = int(360/angular_ring_resolution)                                         # número de puntos por anillo
    theta = np.linspace(0,2*np.pi,n_data)                                             # Ángulo de lidar de 0 a 2PI
    tX = tX                                                                           # Posición en el eje x
    tY = tY                                                                           # Posición en el eje y
    tZ = tZ                                                                           # Posición en el eje z
    pcd_scan = o3d.geometry.PointCloud()
    target_ring = o3d.utility.DoubleVector()
    target_p = o3d.utility.DoubleVector()

    for ring in range(0,len(phi)):                                                    # Generación de los puntos del lidar en el plano X, Y, Z
        for ang_horizontal in range(0,len(theta)):
            # Ecuaciones del modelo Lidar
            P = (-tZ) / (np.cos(alpha)*np.cos(phi[ring]) - np.sin(alpha)*np.sin(phi[ring])*np.cos(theta[ang_horizontal]))
            if(P >= 0 and P <= 100):                                                               # Limitador para el infinito
                x = np.cos(alpha)*(P*np.sin(phi[ring])*np.cos(theta[ang_horizontal])) + np.sin(alpha)*P*np.cos(phi[ring]) + tX
                y = P*np.sin(phi[ring])*np.sin(theta[ang_horizontal]) + tY
                z = np.cos(alpha)*P*np.cos(phi[ring]) - np.sin(alpha)*P*np.sin(phi[ring])*np.cos(theta[ang_horizontal]) + tZ

                # Matriz [x, y, z, ring, P]
                pcd_scan.points.append([x, y, z])
                target_p.append(P)
                target_ring.append(ring)

    points_matrix = transform_open3d_numpy(pcd_scan, target_ring, target_p)
    Impresion_puntos(points_matrix, lidar_fixed_ang, 0, 0)
    return pcd_scan, target_ring, target_p, lidar_fixed_ang

def recorrido(width, length, starting_point, step_limit_x, step_limit_y, dis_furrows, waypoint_distance):
    #variables
    furrow = True
    num_furrow = 0
    distance = 0
    even = 0
    p_x_f = dis_furrows + starting_point[0]
    p_x_i = starting_point[0]
    cont = 0
    scannig_route_matrix = np.empty((0,3),int)
    route_matrix = np.empty((0,3),int)

    if(waypoint_distance < 1):
        waypoint_distance = round(waypoint_distance, 1)
        DISTANCIA = 0.1
    else:
        DISTANCIA = 1
    while(furrow):
        if(num_furrow == 0):
            points_y = starting_point[1]
            while(points_y <= step_limit_y[1]):
                if(points_y == starting_point[1]):
                    route_matrix = np.append(route_matrix, np.array([[starting_point[0], points_y, 0]]), axis = 0)
                if(distance == waypoint_distance):
                    scannig_route_matrix = np.append(scannig_route_matrix, np.array([[starting_point[0], points_y, 0]]), axis = 0)
                    distance = 0
                if(points_y == step_limit_y[1]): break
                distance += DISTANCIA
                points_y += DISTANCIA
                if(DISTANCIA == 0.1):
                    distance = round(distance, 1)
                    points_y = round(points_y, 1)
        if((num_furrow > 1) and (even == 0)):
            while(points_y <= step_limit_y[1]):
                if(distance == waypoint_distance):
                    if(points_x <= step_limit_x[1]):
                        scannig_route_matrix = np.append(scannig_route_matrix, np.array([[points_x, points_y, 0]]), axis= 0)
                        distance = 0
                if(points_y == step_limit_y[1]): break
                distance += DISTANCIA
                points_y += DISTANCIA
                if(DISTANCIA == 0.1):
                    distance = round(distance, 1)
                    points_y = round(points_y, 1)
        if(even != 0):
            while(points_y >= step_limit_y[0]):
                if(distance == waypoint_distance):
                    if(points_x <= step_limit_x[1]):
                        scannig_route_matrix = np.append(scannig_route_matrix, np.array([[points_x, points_y, 0]]), axis= 0)
                        distance = 0
                if(points_y == step_limit_y[0]): break
                distance += DISTANCIA
                points_y -= DISTANCIA
                if(DISTANCIA == 0.1):
                    distance = round(distance, 1)
                    points_y = round(points_y, 1)
        cont = 0
        p_x_i = num_furrow * dis_furrows + starting_point[0]
        p_x_f = (num_furrow + 1)* dis_furrows + starting_point[0]
        points_x = p_x_i
        while(points_x <= p_x_f ):
            if((points_x == p_x_i) or (points_x == p_x_f)):
                if(points_x<= step_limit_x[1]):
                    route_matrix = np.append(route_matrix, np.array([[points_x, points_y, 0]]), axis = 0)
            if(distance == waypoint_distance):
                    if( points_x <= step_limit_x[1]):
                        scannig_route_matrix = np.append(scannig_route_matrix, np.array([[points_x, points_y, 0]]), axis= 0)
                        distance = 0
            if(cont == dis_furrows):
                num_furrow += 1
                even = num_furrow % 2 
                cont = 0
                break
            distance += DISTANCIA
            points_x += DISTANCIA
            cont += DISTANCIA
            if(DISTANCIA == 0.1):
                distance = round(distance, 1)
                points_x = round(points_x, 1)
                cont = round(cont, 1)
        if(p_x_f >= step_limit_x[1]):
            furrow = False
    print(f"la cantida de escaneos son: {len(scannig_route_matrix[:,0])}")
    Impresion_recorrido(scannig_route_matrix, route_matrix, width, length)
    return scannig_route_matrix

def transformada_del_plano(scannig_route_matrix, pcd_scan, target_ring, target_p):
    pcd_scans = o3d.geometry.PointCloud()
    target_rings = o3d.utility.DoubleVector()
    target_ps = o3d.utility.DoubleVector()

    translation_matrix = np.eye(4)

    for route_point in scannig_route_matrix:
        route_x, route_y, route_z = route_point

        translation_matrix[0, 3] = route_x
        translation_matrix[1, 3] = route_y
        translation_matrix[2, 3] = route_z

        pcd_scan_tmp = o3d.geometry.PointCloud()
        pcd_scan_tmp.points.extend(pcd_scan.points)
        pcd_scan_tmp.transform(translation_matrix)

        target_rings.extend(target_ring)
        target_ps.extend(target_p)

        pcd_scans.points.extend(pcd_scan_tmp.points)

    return pcd_scans, target_rings, target_ps
    
def transform_open3d_numpy(pcd_scans, target_rings, target_ps):
    transform_matriz_points = np.asarray(pcd_scans.points)
    points_matrix = np.empty((len(transform_matriz_points),5))
    points_matrix[:,:3] = transform_matriz_points

    ring_matrix = np.asarray(target_rings)
    points_matrix[:,3] = ring_matrix.T
    
    p_matrix = np.asarray(target_ps)
    points_matrix[:,4] = p_matrix.T

    pcd_scans = None
    target_rings = None
    target_ps = None
    transform_matriz_points = None

    return points_matrix

def points_filtered(points_matrix, width, length, lidar_fixed_ang, alpha):
    Xmax = width
    Xmin = -1
    Ymax = length
    Ymin = -1

    pandas_matrix = pd.DataFrame({"x":points_matrix[:,0],"y":points_matrix[:,1],"z":points_matrix[:,2],"ring":points_matrix[:,3],"p":points_matrix[:,4]})

    points_matrix = None
    
    result = pandas_matrix[pandas_matrix["x"] > Xmax].index
    result1 = pandas_matrix[pandas_matrix["x"] < Xmin].index
    result2 = pandas_matrix[pandas_matrix["y"] < Ymin].index
    result3 = pandas_matrix[pandas_matrix["y"] > Ymax].index
    result.union(result1)
    result.union(result2)
    result.union(result3)

    pandas_matrix = pandas_matrix.drop((result))

    points_matrix = pandas_matrix.to_numpy()

    result = None
    result1 = None
    result2 = None
    result3 = None
    pandas_matrix = None

    # pcd = o3d.geometry.PointCloud()
    # pcd.points = o3d.utility.Vector3dVector(points_matrix[:,:3])

    points_m2 = len(points_matrix[:,0]) / (length * width)               # numero de puntos por m^2
    print(f"numero de puntos totales por m^2: {points_m2}")
    average_distance = np.mean( points_matrix,axis = 0)
    print(f"la distancia promedio del lidar a los puntos es: {average_distance[4]}")
    average_ang = (np.sum(lidar_fixed_ang + alpha)) / 8
    print(f"el angulo de incidencia promedio del lidar a los puntos es: {average_ang}")

    # o3d.visualization.draw_geometries([pcd])
    Impresion_puntos(points_matrix, lidar_fixed_ang, width, length)

if __name__ == "__main__":
    # Variables de entrada
    alpha = 0                                                       # Ángulo del lidar
    tX = 0                                                          # Desplazamiento del lidar en el eje x
    tY = 0                                                          # Desplazamiento del lidar en el eje x
    tZ = 2                                                          # Altura del lidar
    width = 100                                                     # width del terreno eje: X
    length = 100                                                    # width del terreno eje: Y
    starting_point = [5,5]                                          # Es el punto de partifa en los ejes (x,y)
    step_limit_x = [10,width-10]                                    # Es el limite minimo y maximo del eje X en el recorrido [min, max]
    step_limit_y = [20,length-20]                                   # Es el limite minimo y maximo del eje y en el recorrido [min, max]
    dis_furrows = 10                                                # La distancia entre furrows en metros
    platform_speed = 400
    freq_scan = 10                                                  # Frecuencia de muestreo(HZ)
    scan_time = 1 / freq_scan                                       # timepo de la ruta : 1/frecuencia_lidar
    waypoint_distance = platform_speed * scan_time                  # Distacia entre puntos
    

    #funciones
    pcd_scan, target_ring, target_p, lidar_fixed_ang = Proyeccion_angular_lidar(alpha, tX, tY, tZ)
    scannig_route_matrix = recorrido(width, length, starting_point, step_limit_x, step_limit_y, dis_furrows, waypoint_distance)
    pcd_scans, target_rings, target_ps = transformada_del_plano(scannig_route_matrix, pcd_scan, target_ring, target_p)
    points_matrix = transform_open3d_numpy(pcd_scans, target_rings, target_ps)
    points_filtered(points_matrix, width, length, lidar_fixed_ang, alpha)
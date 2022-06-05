import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d


def Proyeccion_angular_lidar(alpha, tX, tY, tZ):
    # Variables--------------------------------------
    # Ángulo de inclinación del pilotpcd_scan
    alpha = np.radians(alpha)
    # Ángulo del lidar
    lidar_fixed_ang = np.array(
        [-17, -14.17, -11.33, -8.50, -5.67, -2.83, 0, 3])
    # Traslación del ángulo del lidar al modelo matemático
    gra_phi = 90 - lidar_fixed_ang
    # Transformación de grados a radianes
    phi = np.radians(gra_phi)
    # Resolucion por anillo
    angular_ring_resolution = 0.06
    # número de puntos por anillo
    n_data = int(360/angular_ring_resolution)
    # Ángulo de lidar de 0 a 2PI
    theta = np.linspace(0, 2*np.pi, n_data)
    # Posición en el eje x
    tX = tX
    # Posición en el eje y
    tY = tY
    # Posición en el eje z
    tZ = tZ

    pcd_scan = o3d.geometry.PointCloud()
    target_ring = o3d.utility.DoubleVector()
    target_p = o3d.utility.DoubleVector()

    # Generación de los puntos del lidar en el plano X, Y, Z
    for ring in range(0, len(phi)):
        for ang_horizontal in range(0, len(theta)):
            # Ecuaciones del modelo Lidar
            P = (-tZ) / (np.cos(alpha)*np.cos(phi[ring]) - np.sin(
                alpha)*np.sin(phi[ring])*np.cos(theta[ang_horizontal]))
            # Limitador para el infinito
            if(P >= 0 and P <= 100):
                x = np.cos(alpha)*(P*np.sin(phi[ring])*np.cos(
                    theta[ang_horizontal])) + np.sin(alpha)*P*np.cos(phi[ring]) + tX
                y = P*np.sin(phi[ring])*np.sin(theta[ang_horizontal]) + tY
                z = np.cos(alpha)*P*np.cos(phi[ring]) - np.sin(alpha) * \
                    P*np.sin(phi[ring])*np.cos(theta[ang_horizontal]) + tZ

                # Matriz [x, y, z, ring, P]
                pcd_scan.points.append([x, y, z])
                target_p.append(P)
                target_ring.append(ring)

    return pcd_scan, target_ring, target_p, lidar_fixed_ang


def Impresion_recorrido(df_p, df_r, width, length):
    # Imprimir en pantalla
    fig, ax = plt.subplots()
    ax.set_title("3d meassure plane", fontsize=14, fontweight="bold")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_xlim(0, width)
    ax.set_ylim(0, length)
    ax.plot(df_p[:, 0].T, df_p[:, 1].T, "x")
    ax.plot(df_r[:, 0].T, df_r[:, 1].T)
    plt.tight_layout()
    plt.show()


def recorrido(starting_point, step_limit_x, step_limit_y, dis_furrows, waypoint_distance):
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
        print(num_furrow)
    print(f"la cantida de escaneos son: {len(scannig_route_matrix[:,0])}")
    Impresion_recorrido(scannig_route_matrix, route_matrix, width, length)
    return scannig_route_matrix


def transformada_del_plano(scannig_route_matrix, pcd_scan, target_ring, target_p):
    pcd_scans = o3d.geometry.PointCloud()
    target_rings = o3d.utility.DoubleVector()
    target_ps = o3d.utility.DoubleVector()

    pcd_ruta = o3d.geometry.PointCloud()
    pcd_ruta.points = o3d.utility.Vector3dVector(scannig_route_matrix)
    pcd_ruta.paint_uniform_color([1, 0, 0])

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
    
    
    o3d.visualization.draw_geometries([pcd_ruta, pcd_scans])

    return pcd_scans, target_rings, target_ps


if __name__ == "__main__":
    # Ángulo del lidar
    alpha = 0
    # Desplazamiento del lidar en el eje x
    tX = 0
    # Desplazamiento del lidar en el eje x
    tY = 0
    tZ = 2                                                          # Altura del lidar
    # width del terreno eje: X
    width = 100
    # width del terreno eje: Y
    length = 100
    # Es el punto de partifa en los ejes (x,y)
    starting_point = [5, 5]
    # Es el limite minimo y maximo del eje X en el recorrido [min, max]
    step_limit_x = [10, width-10]
    # Es el limite minimo y maximo del eje y en el recorrido [min, max]
    step_limit_y = [20, length-20]
    # La distancia entre furrows en metros
    dis_furrows = 10
    # Es el numero de furrows que hay en el cultivo
    platform_speed = 3
    # Frecuencia de muestreo(HZ)
    freq_scan = 10
    # timepo de la ruta : 1/frecuencia_lidar
    scan_time = 1 / freq_scan
    waypoint_distance = platform_speed * scan_time

    # funciones
    pcd_scan, target_ring, target_p, lidar_fixed_ang = Proyeccion_angular_lidar(
        alpha, tX, tY, tZ)

    scannig_route_matrix = recorrido(
        starting_point, step_limit_x, step_limit_y, dis_furrows, waypoint_distance)

    pcd_scans, target_rings, target_ps = transformada_del_plano(
        scannig_route_matrix, pcd_scan, target_ring, target_p)

    a = np.asarray(pcd_scans.points)
    print(a)

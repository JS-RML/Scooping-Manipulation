#!/usr/bin/env python
import numpy as np
from math import *
from scipy.optimize import linprog
import os
from general_functions import *
from shapely.geometry import LineString
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches

env_profile = [[0,sqrt(3)], [1,0], [3,0]]  # thumb and ground
object_vertex_position_on_ground_0 = [[1,0], [2.5,0], [2.5,0.3], [1,0.3]]
object_vertex_position_on_ground_1 = [[2.5, 0.0], [2.49375, 0.038649062084350684], [2.4875, 0.05454356057317852], [2.48125, 0.06666145812986687], [2.475, 0.07681145747868597], [2.46875, 0.08569568250501293], [2.4625000000000004, 0.09367496997597582], [2.4562500000000003, 0.10096410253154318], [2.45, 0.10770329614268986], [2.44375, 0.11399013115177982], [2.4375, 0.11989578808281777], [2.4312500000000004, 0.1254741009132958], [2.4250000000000003, 0.13076696830622], [2.41875, 0.1358077685554105], [2.4125000000000005, 0.1406236111042521], [2.4062500000000004, 0.1452368754827779], [2.4000000000000004, 0.14966629547095742], [2.3937500000000003, 0.15392774278862115], [2.3875, 0.1580348062927908], [2.3812500000000005, 0.16199922839322387], [2.3750000000000004, 0.1658312395177697], [2.3687500000000004, 0.16953981833185944], [2.3625000000000007, 0.17313289693180758], [2.3562500000000006, 0.17661752461180033], [2.3500000000000005, 0.17999999999999972], [2.3437500000000004, 0.18328597873268945], [2.3375000000000004, 0.18648056198971488], [2.3312500000000007, 0.18958836989646774], [2.3250000000000006, 0.19261360284258192], [2.3187500000000005, 0.19556009306604424], [2.312500000000001, 0.198431348329844], [2.306250000000001, 0.20123058912600705], [2.3000000000000007, 0.20396078054371108], [2.2937500000000006, 0.20662465970933833], [2.2875000000000005, 0.20922475952907646], [2.281250000000001, 0.2117634293262174], [2.275000000000001, 0.21424285285628517], [2.2687500000000007, 0.2166650640966371], [2.262500000000001, 0.21903196113809478], [2.256250000000001, 0.2213453184506053], [2.250000000000001, 0.22360679774997866], [2.243750000000001, 0.22581795765616128], [2.2375000000000007, 0.22798026230355964], [2.231250000000001, 0.2300950890392923], [2.225000000000001, 0.23216373532487766], [2.218750000000001, 0.2341874249399396], [2.2125000000000012, 0.2361673135723905], [2.206250000000001, 0.23810449386771312], [2.200000000000001, 0.23999999999999966], [2.193750000000001, 0.24185481181899163], [2.187500000000001, 0.24366985862022378], [2.1812500000000012, 0.24544602257930323], [2.175000000000001, 0.24718414188616517], [2.168750000000001, 0.24888501361070306], [2.1625000000000014, 0.2505493963273507], [2.1562500000000013, 0.25217801252289984], [2.1500000000000012, 0.25377155080899005], [2.143750000000001, 0.2553306679582377], [2.137500000000001, 0.2568559907808261], [2.1312500000000014, 0.2583481178565074], [2.1250000000000013, 0.2598076211353313], [2.1187500000000012, 0.26123504741898596], [2.1125000000000016, 0.2626309197333776], [2.1062500000000015, 0.2639957386019703], [2.1000000000000014, 0.26532998322843165], [2.0937500000000013, 0.26663411259626896], [2.0875000000000012, 0.26790856649237593], [2.0812500000000016, 0.2691537664607349], [2.0750000000000015, 0.2703701166919152], [2.0687500000000014, 0.2715580048534748], [2.0625000000000018, 0.2727178028658926], [2.0562500000000017, 0.2738498676282314], [2.0500000000000016, 0.2749545416973501], [2.0437500000000015, 0.2760321539241395], [2.0375000000000014, 0.27708302004994795], [2.0312500000000018, 0.2781074432660871], [2.0250000000000017, 0.279105714739057], [2.0187500000000016, 0.28007811410390465], [2.012500000000002, 0.28102490992792767], [2.006250000000002, 0.28194636014674823], [2.0000000000000018, 0.28284271247461873], [1.9937500000000017, 0.2837142047906658], [1.9875000000000018, 0.28456106550264365], [1.981250000000002, 0.28538351388964267], [1.9750000000000019, 0.2861817604250835], [1.9687500000000018, 0.28695600708122465], [1.962500000000002, 0.2877064476163158], [1.956250000000002, 0.28843326784544093], [1.950000000000002, 0.289136645896019], [1.9437500000000019, 0.28981675244885324], [1.937500000000002, 0.29047375096555605], [1.9312500000000021, 0.2911077979031133], [1.925000000000002, 0.2917190429162962], [1.918750000000002, 0.29230762904857593], [1.912500000000002, 0.2928736929121492], [1.9062500000000022, 0.29341736485763736], [1.9000000000000021, 0.2939387691339812], [1.893750000000002, 0.29443802403901553], [1.8875000000000022, 0.29491524206117237], [1.8812500000000023, 0.29537053001272806], [1.8750000000000022, 0.29580398915498063], [1.8687500000000021, 0.2962157153157136], [1.8625000000000023, 0.2966057989992777], [1.8562500000000024, 0.296974325489595], [1.8500000000000023, 0.29732137494636995], [1.8437500000000022, 0.2976470224947663], [1.8375000000000024, 0.2979513383087915], [1.8312500000000025, 0.29823438768860966], [1.8250000000000024, 0.29849623113198587], [1.8187500000000023, 0.2987369244000479], [1.8125000000000024, 0.29895651857753486], [1.8062500000000026, 0.2991550601276869], [1.8000000000000025, 0.2993325909419152], [1.7937500000000024, 0.29948914838437796], [1.7875000000000025, 0.2996247653315726], [1.7812500000000027, 0.2997394702070449], [1.7750000000000026, 0.29983328701129897], [1.7687500000000025, 0.299906235346983], [1.7625000000000026, 0.29995833043941283], [1.7562500000000028, 0.29998958315248214], [1.75, 0.3], [1.75, 0.3], [1.74375, 0.29998958315248214], [1.7375, 0.29995833043941283], [1.73125, 0.2999062353469831], [1.725, 0.29983328701129897], [1.71875, 0.29973947020704494], [1.7125, 0.2996247653315727], [1.70625, 0.299489148384378], [1.7, 0.29933259094191533], [1.69375, 0.29915506012768694], [1.6875, 0.29895651857753497], [1.68125, 0.298736924400048], [1.675, 0.298496231131986], [1.66875, 0.29823438768860977], [1.6625, 0.2979513383087916], [1.65625, 0.29764702249476643], [1.65, 0.2973213749463701], [1.64375, 0.2969743254895951], [1.6375, 0.29660579899927786], [1.63125, 0.2962157153157138], [1.625, 0.2958039891549808], [1.61875, 0.2953705300127282], [1.6125, 0.29491524206117253], [1.60625, 0.2944380240390157], [1.6, 0.29393876913398137], [1.59375, 0.29341736485763753], [1.5875, 0.29287369291214943], [1.58125, 0.29230762904857616], [1.575, 0.2917190429162964], [1.56875, 0.29110779790311353], [1.5625, 0.2904737509655563], [1.55625, 0.28981675244885347], [1.55, 0.2891366458960192], [1.54375, 0.28843326784544115], [1.5375, 0.28770644761631603], [1.53125, 0.28695600708122493], [1.525, 0.2861817604250837], [1.51875, 0.2853835138896429], [1.5125, 0.28456106550264393], [1.50625, 0.28371420479066606], [1.5, 0.282842712474619], [1.49375, 0.28194636014674845], [1.4875, 0.2810249099279279], [1.48125, 0.28007811410390493], [1.475, 0.2791057147390572], [1.46875, 0.27810744326608733], [1.4625, 0.2770830200499482], [1.45625, 0.27603215392413977], [1.45, 0.2749545416973504], [1.44375, 0.2738498676282316], [1.4375, 0.27271780286589287], [1.43125, 0.2715580048534751], [1.425, 0.27037011669191546], [1.41875, 0.2691537664607352], [1.4125, 0.26790856649237627], [1.40625, 0.2666341125962693], [1.4, 0.265329983228432], [1.39375, 0.2639957386019706], [1.3875, 0.2626309197333779], [1.38125, 0.26123504741898623], [1.375, 0.25980762113533157], [1.36875, 0.25834811785650774], [1.3625, 0.25685599078082644], [1.35625, 0.255330667958238], [1.35, 0.2537715508089904], [1.34375, 0.2521780125229002], [1.3375, 0.25054939632735096], [1.33125, 0.24888501361070337], [1.325, 0.2471841418861655], [1.31875, 0.24544602257930356], [1.3125, 0.24366985862022408], [1.30625, 0.24185481181899188], [1.3, 0.23999999999999996], [1.29375, 0.23810449386771343], [1.2875, 0.23616731357239085], [1.28125, 0.23418742493993994], [1.275, 0.23216373532487797], [1.26875, 0.23009508903929263], [1.2625, 0.22798026230355994], [1.25625, 0.22581795765616158], [1.25, 0.22360679774997896], [1.24375, 0.2213453184506056], [1.2374999999999998, 0.21903196113809506], [1.23125, 0.21666506409663738], [1.225, 0.2142428528562855], [1.21875, 0.2117634293262177], [1.2125, 0.2092247595290768], [1.2062499999999998, 0.20662465970933863], [1.2, 0.20396078054371133], [1.19375, 0.20123058912600736], [1.1875, 0.1984313483298443], [1.18125, 0.19556009306604455], [1.1749999999999998, 0.1926136028425822], [1.16875, 0.18958836989646805], [1.1625, 0.18648056198971513], [1.15625, 0.18328597873268973], [1.15, 0.17999999999999997], [1.1437499999999998, 0.1766175246118006], [1.1375, 0.1731328969318078], [1.13125, 0.16953981833185972], [1.125, 0.16583123951777], [1.11875, 0.16199922839322406], [1.1124999999999998, 0.15803480629279104], [1.10625, 0.1539277427886214], [1.1, 0.14966629547095764], [1.09375, 0.14523687548277814], [1.0875, 0.14062361110425226], [1.0812499999999998, 0.13580776855541066], [1.075, 0.13076696830622017], [1.06875, 0.12547410091329605], [1.0625, 0.11989578808281798], [1.05625, 0.11399013115177985], [1.0499999999999998, 0.10770329614269], [1.04375, 0.10096410253154334], [1.0375, 0.09367496997597594], [1.03125, 0.08569568250501304], [1.025, 0.07681145747868597], [1.0187499999999998, 0.06666145812986687], [1.0125, 0.05454356057317852], [1.00625, 0.038649062084350684], [1, 0]]
object_vertex_position_on_ground_2 = [[1, 0], [2.5, 0], [2.5, 0.29945012105524355], [2.45, 0.28460308786975597], [2.4000000000000004, 0.27000988130066655], [2.3500000000000005, 0.25567499559988277], [2.3000000000000007, 0.24160317257850755], [2.250000000000001, 0.2277994252077913], [2.200000000000001, 0.2142690644960213], [2.1500000000000012, 0.20101773025034417], [2.1000000000000014, 0.18805142647690853], [2.0500000000000016, 0.17537656235939894], [2.0000000000000018, 0.16300000000000045], [1.950000000000002, 0.15092911042936727], [1.9000000000000021, 0.13917183982401088], [1.8500000000000023, 0.12773678845579348], [1.8000000000000025, 0.11663330570638958], [1.7500000000000027, 0.10587160561264819], [1.7000000000000028, 0.09546290902753861], [1.650000000000003, 0.08541962084322373], [1.6000000000000032, 0.07575555425181768], [1.5500000000000034, 0.06648621943681324], [1.5000000000000036, 0.05762920266670424], [1.4500000000000037, 0.04920467584488348], [1.400000000000004, 0.041236100688596275], [1.350000000000004, 0.0337512351625839], [1.3000000000000043, 0.026783633062003195], [1.2500000000000044, 0.020375000000000542], [1.2000000000000046, 0.014579163213299134], [1.1500000000000048, 0.009469444281477588], [1.100000000000005, 0.005154512586074843], [1.0500000000000052, 0.0018223954016626104]]
object_vertex_position_on_ground_3 = [[2.449999999999995, 0.0018223954016626104], [2.399999999999995, 0.005154512586074843], [2.349999999999995, 0.009469444281477588], [2.2999999999999954, 0.014579163213299134], [2.2499999999999956, 0.020375000000000542], [2.1999999999999957, 0.026783633062003195], [2.149999999999996, 0.0337512351625839], [2.099999999999996, 0.041236100688596275], [2.0499999999999963, 0.04920467584488348], [1.9999999999999964, 0.05762920266670424], [1.9499999999999966, 0.06648621943681324], [1.8999999999999968, 0.07575555425181768], [1.849999999999997, 0.08541962084322373], [1.7999999999999972, 0.09546290902753861], [1.7499999999999973, 0.10587160561264819], [1.6999999999999975, 0.11663330570638958], [1.6499999999999977, 0.12773678845579348], [1.5999999999999979, 0.13917183982401088], [1.549999999999998, 0.15092911042936727], [1.4999999999999982, 0.16300000000000045], [1.4499999999999984, 0.17537656235939894], [1.3999999999999986, 0.18805142647690853], [1.3499999999999988, 0.20101773025034417], [1.299999999999999, 0.2142690644960213], [1.2499999999999991, 0.2277994252077913], [1.1999999999999993, 0.24160317257850755], [1.1499999999999995, 0.25567499559988277], [1.0999999999999996, 0.27000988130066655], [1.0499999999999998, 0.28460308786975597], [1.0, 0.29945012105524355], [1.0, 0], [2.5, 0]]
phi = 10
for object_vertex_position_on_ground in [object_vertex_position_on_ground_0, object_vertex_position_on_ground_1, object_vertex_position_on_ground_2, object_vertex_position_on_ground_3]:
    object_vertex_position_after_rot = [point_position_after_rotation(point, [0,0], -phi) for point in object_vertex_position_on_ground]
    lowest_point = object_vertex_position_after_rot[0]
    for point in object_vertex_position_after_rot:
        if point[1]<=lowest_point[1]:
            lowest_point = point
    object_vertex_position_after_rot = [[point[0], point[1]-lowest_point[1]] for point in object_vertex_position_after_rot]
    print object_vertex_position_after_rot
    leftmost_distance_x_set = []
    for point in object_vertex_position_after_rot:
        if LineString([[-100, point[1]], [100, point[1]]]).intersects(LineString([[0,sqrt(3)], [1,0]]))==True:
            intersection_point = list(LineString([[-100, point[1]], [100, point[1]]]).intersection(LineString([[0,sqrt(3)], [1,0]])).coords[0])
            leftmost_distance_x_set.append(point[0]-intersection_point[0])
    print leftmost_distance_x_set
    object_vertex_position = [[point[0]-min(leftmost_distance_x_set), point[1]] for point in object_vertex_position_after_rot]

    feasible_finger_position_set = []
    for i in range(1,101):
        point_index_temp = (i-0.5)*(circumference(object_vertex_position)/100.0)
        finger_position = finger_index2position(object_vertex_position, point_index_temp)
        contact_index, contact_list, contact_normal_list, contact_local_tangent_list = contact_points_and_normal(object_vertex_position, finger_position, env_profile)
        A_ub = np.array(contact_normal_list)
        A_ub = A_ub[:, [1, 0]]
        A_ub[:, 0] = -A_ub[:, 0]
        A_ub = A_ub.tolist()
        b_ub = []
        for k in range(len(contact_list)):
            b_ub.append(-contact_list[k][0]*contact_normal_list[k][1]+contact_list[k][1]*contact_normal_list[k][0])
        f = [1,1]
        res1 = linprog(f, A_ub, b_ub)
        if res1.status == 2:
            Rotate_CW = False
        else:
            A_ub = (-np.array(A_ub)).tolist()
            b_ub = (-np.array(b_ub)).tolist()
            res2 = linprog(f, A_ub, b_ub)
            if res2.status !=2:
                Rotate_CW = False
            else:
                Rotate_CW = True
        if Rotate_CW == True:
            feasible_finger_position_set.append(finger_position)

    print feasible_finger_position_set

    fig = plt.figure()
    ax = fig.add_subplot(111, autoscale_on=False, xlim=(-0.1, 3.1), ylim=(-0.1, 2.1))
    ax.set_aspect('equal')
    env_ground = [[0,0], [3,0]]
    env_thumb = [[0,sqrt(3)], [1,0]]
    env_plot0, = ax.plot([i[0] for i in env_ground], [i[1] for i in env_ground], 'grey', lw=2, zorder=-1) 
    env_plot1, = ax.plot([i[0] for i in env_thumb], [i[1] for i in env_thumb], 'grey', lw=1.5, zorder=-1) 
    verts=[tuple(k) for k in object_vertex_position]
    verts.append(tuple(object_vertex_position[0]))
    codes = [Path.MOVETO]
    for j in range(len(verts)-2):
        codes.append(Path.LINETO)
    codes.append(Path.CLOSEPOLY)
    path=Path(verts,codes)
    patch = patches.PathPatch(path, facecolor='darkgrey', edgecolor='black', lw=2, zorder=0)
    ax.add_patch(patch)
    for point in feasible_finger_position_set:
        feasible_finger_position, = ax.plot([point[0]], [point[1]], '-o', color='blue', zorder=5, lw=2, markersize=5)
    plt.show()


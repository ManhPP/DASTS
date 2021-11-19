\ Model DASTS-GUROBI-V2
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  D[1,0] + D[2,0] + D[3,0] + D[4,0] + D[5,0] + D[6,0] + D[1,1] + D[2,1]
   + D[3,1] + D[4,1] + D[5,1] + D[6,1] + 0 t[1] + 0 t_a[1] + 0 T[1]
   + 0 t[2] + 0 t_a[2] + 0 T[2] + 0 t[3] + 0 t_a[3] + 0 T[3] + 0 t[4]
   + 0 t_a[4] + 0 T[4] + 0 t[5] + 0 t_a[5] + 0 T[5] + 0 t[6] + 0 t_a[6]
   + 0 T[6] + 0 t[7] + 0 t_a[7] + 0 T[7] + 0 t_a[0] + 0 T[0] + 0 B[0]
   + 0 B[1] + 0 tmp_sample_tech_temp[0] + 0 tmp_sample_tech_temp[1]
Subject To
 inoutDepotTech_tech0: x[0,1,0] + x[0,2,0] + x[0,3,0] + x[0,4,0] + x[0,5,0]
   + x[0,6,0] - x[1,7,0] - x[2,7,0] - x[3,7,0] - x[4,7,0] - x[5,7,0]
   - x[6,7,0] = 0
 outDepotTech_tech0: x[0,1,0] + x[0,2,0] + x[0,3,0] + x[0,4,0] + x[0,5,0]
   + x[0,6,0] + x[0,7,0] <= 1
 inoutDepotTech_tech1: x[0,1,1] + x[0,2,1] + x[0,3,1] + x[0,4,1] + x[0,5,1]
   + x[0,6,1] - x[1,7,1] - x[2,7,1] - x[3,7,1] - x[4,7,1] - x[5,7,1]
   - x[6,7,1] = 0
 outDepotTech_tech1: x[0,1,1] + x[0,2,1] + x[0,3,1] + x[0,4,1] + x[0,5,1]
   + x[0,6,1] + x[0,7,1] <= 1
 inoutCusTech_node1_tech0: - x[2,1,0] - x[3,1,0] - x[4,1,0] - x[5,1,0]
   - x[6,1,0] - x[0,1,0] + x[1,2,0] + x[1,3,0] + x[1,4,0] + x[1,5,0]
   + x[1,6,0] + x[1,7,0] = 0
 inoutCusTech_node1_tech1: - x[2,1,1] - x[3,1,1] - x[4,1,1] - x[5,1,1]
   - x[6,1,1] - x[0,1,1] + x[1,2,1] + x[1,3,1] + x[1,4,1] + x[1,5,1]
   + x[1,6,1] + x[1,7,1] = 0
 inoutCusTech_node2_tech0: x[2,1,0] - x[1,2,0] - x[3,2,0] - x[4,2,0]
   - x[5,2,0] - x[6,2,0] - x[0,2,0] + x[2,3,0] + x[2,4,0] + x[2,5,0]
   + x[2,6,0] + x[2,7,0] = 0
 inoutCusTech_node2_tech1: x[2,1,1] - x[1,2,1] - x[3,2,1] - x[4,2,1]
   - x[5,2,1] - x[6,2,1] - x[0,2,1] + x[2,3,1] + x[2,4,1] + x[2,5,1]
   + x[2,6,1] + x[2,7,1] = 0
 inoutCusTech_node3_tech0: x[3,1,0] + x[3,2,0] - x[1,3,0] - x[2,3,0]
   - x[4,3,0] - x[5,3,0] - x[6,3,0] - x[0,3,0] + x[3,4,0] + x[3,5,0]
   + x[3,6,0] + x[3,7,0] = 0
 inoutCusTech_node3_tech1: x[3,1,1] + x[3,2,1] - x[1,3,1] - x[2,3,1]
   - x[4,3,1] - x[5,3,1] - x[6,3,1] - x[0,3,1] + x[3,4,1] + x[3,5,1]
   + x[3,6,1] + x[3,7,1] = 0
 inoutCusTech_node4_tech0: x[4,1,0] + x[4,2,0] + x[4,3,0] - x[1,4,0]
   - x[2,4,0] - x[3,4,0] - x[5,4,0] - x[6,4,0] - x[0,4,0] + x[4,5,0]
   + x[4,6,0] + x[4,7,0] = 0
 inoutCusTech_node4_tech1: x[4,1,1] + x[4,2,1] + x[4,3,1] - x[1,4,1]
   - x[2,4,1] - x[3,4,1] - x[5,4,1] - x[6,4,1] - x[0,4,1] + x[4,5,1]
   + x[4,6,1] + x[4,7,1] = 0
 inoutCusTech_node5_tech0: x[5,1,0] + x[5,2,0] + x[5,3,0] + x[5,4,0]
   - x[1,5,0] - x[2,5,0] - x[3,5,0] - x[4,5,0] - x[6,5,0] - x[0,5,0]
   + x[5,6,0] + x[5,7,0] = 0
 inoutCusTech_node5_tech1: x[5,1,1] + x[5,2,1] + x[5,3,1] + x[5,4,1]
   - x[1,5,1] - x[2,5,1] - x[3,5,1] - x[4,5,1] - x[6,5,1] - x[0,5,1]
   + x[5,6,1] + x[5,7,1] = 0
 inoutCusTech_node6_tech0: x[6,1,0] + x[6,2,0] + x[6,3,0] + x[6,4,0]
   + x[6,5,0] - x[1,6,0] - x[2,6,0] - x[3,6,0] - x[4,6,0] - x[5,6,0]
   - x[0,6,0] + x[6,7,0] = 0
 inoutCusTech_node6_tech1: x[6,1,1] + x[6,2,1] + x[6,3,1] + x[6,4,1]
   + x[6,5,1] - x[1,6,1] - x[2,6,1] - x[3,6,1] - x[4,6,1] - x[5,6,1]
   - x[0,6,1] + x[6,7,1] = 0
 cusOnceTech_node1: x[1,2,0] + x[1,3,0] + x[1,4,0] + x[1,5,0] + x[1,6,0]
   + x[1,7,0] + x[1,2,1] + x[1,3,1] + x[1,4,1] + x[1,5,1] + x[1,6,1]
   + x[1,7,1] = 1
 cusOnceTech_node2: x[2,1,0] + x[2,3,0] + x[2,4,0] + x[2,5,0] + x[2,6,0]
   + x[2,7,0] + x[2,1,1] + x[2,3,1] + x[2,4,1] + x[2,5,1] + x[2,6,1]
   + x[2,7,1] = 1
 cusOnceTech_node3: x[3,1,0] + x[3,2,0] + x[3,4,0] + x[3,5,0] + x[3,6,0]
   + x[3,7,0] + x[3,1,1] + x[3,2,1] + x[3,4,1] + x[3,5,1] + x[3,6,1]
   + x[3,7,1] = 1
 cusOnceTech_node4: x[4,1,0] + x[4,2,0] + x[4,3,0] + x[4,5,0] + x[4,6,0]
   + x[4,7,0] + x[4,1,1] + x[4,2,1] + x[4,3,1] + x[4,5,1] + x[4,6,1]
   + x[4,7,1] = 1
 cusOnceTech_node5: x[5,1,0] + x[5,2,0] + x[5,3,0] + x[5,4,0] + x[5,6,0]
   + x[5,7,0] + x[5,1,1] + x[5,2,1] + x[5,3,1] + x[5,4,1] + x[5,6,1]
   + x[5,7,1] = 1
 cusOnceTech_node6: x[6,1,0] + x[6,2,0] + x[6,3,0] + x[6,4,0] + x[6,5,0]
   + x[6,7,0] + x[6,1,1] + x[6,2,1] + x[6,3,1] + x[6,4,1] + x[6,5,1]
   + x[6,7,1] = 1
 tmpTimeAtCusTech1_node1_node2: - x[2,1,0] - x[2,1,1]
   + tmp_time_at_cus_tech1[1,2] = 0
 tmpTimeAtCusTech1_node1_node3: - x[3,1,0] - x[3,1,1]
   + tmp_time_at_cus_tech1[1,3] = 0
 tmpTimeAtCusTech1_node1_node4: - x[4,1,0] - x[4,1,1]
   + tmp_time_at_cus_tech1[1,4] = 0
 tmpTimeAtCusTech1_node1_node5: - x[5,1,0] - x[5,1,1]
   + tmp_time_at_cus_tech1[1,5] = 0
 tmpTimeAtCusTech1_node1_node6: - x[6,1,0] - x[6,1,1]
   + tmp_time_at_cus_tech1[1,6] = 0
 tmpTimeAtCusTech1_node1_node0: - x[0,1,0] - x[0,1,1]
   + tmp_time_at_cus_tech1[1,0] = 0
 tmpTimeAtCusTech1_node2_node1: - x[1,2,0] - x[1,2,1]
   + tmp_time_at_cus_tech1[2,1] = 0
 tmpTimeAtCusTech1_node2_node3: - x[3,2,0] - x[3,2,1]
   + tmp_time_at_cus_tech1[2,3] = 0
 tmpTimeAtCusTech1_node2_node4: - x[4,2,0] - x[4,2,1]
   + tmp_time_at_cus_tech1[2,4] = 0
 tmpTimeAtCusTech1_node2_node5: - x[5,2,0] - x[5,2,1]
   + tmp_time_at_cus_tech1[2,5] = 0
 tmpTimeAtCusTech1_node2_node6: - x[6,2,0] - x[6,2,1]
   + tmp_time_at_cus_tech1[2,6] = 0
 tmpTimeAtCusTech1_node2_node0: - x[0,2,0] - x[0,2,1]
   + tmp_time_at_cus_tech1[2,0] = 0
 tmpTimeAtCusTech1_node3_node1: - x[1,3,0] - x[1,3,1]
   + tmp_time_at_cus_tech1[3,1] = 0
 tmpTimeAtCusTech1_node3_node2: - x[2,3,0] - x[2,3,1]
   + tmp_time_at_cus_tech1[3,2] = 0
 tmpTimeAtCusTech1_node3_node4: - x[4,3,0] - x[4,3,1]
   + tmp_time_at_cus_tech1[3,4] = 0
 tmpTimeAtCusTech1_node3_node5: - x[5,3,0] - x[5,3,1]
   + tmp_time_at_cus_tech1[3,5] = 0
 tmpTimeAtCusTech1_node3_node6: - x[6,3,0] - x[6,3,1]
   + tmp_time_at_cus_tech1[3,6] = 0
 tmpTimeAtCusTech1_node3_node0: - x[0,3,0] - x[0,3,1]
   + tmp_time_at_cus_tech1[3,0] = 0
 tmpTimeAtCusTech1_node4_node1: - x[1,4,0] - x[1,4,1]
   + tmp_time_at_cus_tech1[4,1] = 0
 tmpTimeAtCusTech1_node4_node2: - x[2,4,0] - x[2,4,1]
   + tmp_time_at_cus_tech1[4,2] = 0
 tmpTimeAtCusTech1_node4_node3: - x[3,4,0] - x[3,4,1]
   + tmp_time_at_cus_tech1[4,3] = 0
 tmpTimeAtCusTech1_node4_node5: - x[5,4,0] - x[5,4,1]
   + tmp_time_at_cus_tech1[4,5] = 0
 tmpTimeAtCusTech1_node4_node6: - x[6,4,0] - x[6,4,1]
   + tmp_time_at_cus_tech1[4,6] = 0
 tmpTimeAtCusTech1_node4_node0: - x[0,4,0] - x[0,4,1]
   + tmp_time_at_cus_tech1[4,0] = 0
 tmpTimeAtCusTech1_node5_node1: - x[1,5,0] - x[1,5,1]
   + tmp_time_at_cus_tech1[5,1] = 0
 tmpTimeAtCusTech1_node5_node2: - x[2,5,0] - x[2,5,1]
   + tmp_time_at_cus_tech1[5,2] = 0
 tmpTimeAtCusTech1_node5_node3: - x[3,5,0] - x[3,5,1]
   + tmp_time_at_cus_tech1[5,3] = 0
 tmpTimeAtCusTech1_node5_node4: - x[4,5,0] - x[4,5,1]
   + tmp_time_at_cus_tech1[5,4] = 0
 tmpTimeAtCusTech1_node5_node6: - x[6,5,0] - x[6,5,1]
   + tmp_time_at_cus_tech1[5,6] = 0
 tmpTimeAtCusTech1_node5_node0: - x[0,5,0] - x[0,5,1]
   + tmp_time_at_cus_tech1[5,0] = 0
 tmpTimeAtCusTech1_node6_node1: - x[1,6,0] - x[1,6,1]
   + tmp_time_at_cus_tech1[6,1] = 0
 tmpTimeAtCusTech1_node6_node2: - x[2,6,0] - x[2,6,1]
   + tmp_time_at_cus_tech1[6,2] = 0
 tmpTimeAtCusTech1_node6_node3: - x[3,6,0] - x[3,6,1]
   + tmp_time_at_cus_tech1[6,3] = 0
 tmpTimeAtCusTech1_node6_node4: - x[4,6,0] - x[4,6,1]
   + tmp_time_at_cus_tech1[6,4] = 0
 tmpTimeAtCusTech1_node6_node5: - x[5,6,0] - x[5,6,1]
   + tmp_time_at_cus_tech1[6,5] = 0
 tmpTimeAtCusTech1_node6_node0: - x[0,6,0] - x[0,6,1]
   + tmp_time_at_cus_tech1[6,0] = 0
 leaveDepotTech: t[0] = 0
 getSampleNode1_node1: <= 1
 getSampleNode1_node2: <= 1
 getSampleNode1_node3: <= 1
 getSampleNode1_node4: <= 1
 getSampleNode1_node5: <= 1
 getSampleNode1_node6: <= 1
 getSampleNode3_node1_tech0: - x[1,2,0] - x[1,3,0] - x[1,4,0] - x[1,5,0]
   - x[1,6,0] - x[1,7,0] <= 0
 getSampleNode3_node1_tech1: - x[1,2,1] - x[1,3,1] - x[1,4,1] - x[1,5,1]
   - x[1,6,1] - x[1,7,1] <= 0
 getSampleNode3_node2_tech0: - x[2,1,0] - x[2,3,0] - x[2,4,0] - x[2,5,0]
   - x[2,6,0] - x[2,7,0] <= 0
 getSampleNode3_node2_tech1: - x[2,1,1] - x[2,3,1] - x[2,4,1] - x[2,5,1]
   - x[2,6,1] - x[2,7,1] <= 0
 getSampleNode3_node3_tech0: - x[3,1,0] - x[3,2,0] - x[3,4,0] - x[3,5,0]
   - x[3,6,0] - x[3,7,0] <= 0
 getSampleNode3_node3_tech1: - x[3,1,1] - x[3,2,1] - x[3,4,1] - x[3,5,1]
   - x[3,6,1] - x[3,7,1] <= 0
 getSampleNode3_node4_tech0: - x[4,1,0] - x[4,2,0] - x[4,3,0] - x[4,5,0]
   - x[4,6,0] - x[4,7,0] <= 0
 getSampleNode3_node4_tech1: - x[4,1,1] - x[4,2,1] - x[4,3,1] - x[4,5,1]
   - x[4,6,1] - x[4,7,1] <= 0
 getSampleNode3_node5_tech0: - x[5,1,0] - x[5,2,0] - x[5,3,0] - x[5,4,0]
   - x[5,6,0] - x[5,7,0] <= 0
 getSampleNode3_node5_tech1: - x[5,1,1] - x[5,2,1] - x[5,3,1] - x[5,4,1]
   - x[5,6,1] - x[5,7,1] <= 0
 getSampleNode3_node6_tech0: - x[6,1,0] - x[6,2,0] - x[6,3,0] - x[6,4,0]
   - x[6,5,0] - x[6,7,0] <= 0
 getSampleNode3_node6_tech1: - x[6,1,1] - x[6,2,1] - x[6,3,1] - x[6,4,1]
   - x[6,5,1] - x[6,7,1] <= 0
 tmpGetSampleNode4_node1_tech0: - x[1,2,0] - x[1,3,0] - x[1,4,0] - x[1,5,0]
   - x[1,6,0] - x[1,7,0] + tmp_get_sample_node4[1,0] = 0
 tmpGetSampleNode4_node2_tech0: - x[2,1,0] - x[2,3,0] - x[2,4,0] - x[2,5,0]
   - x[2,6,0] - x[2,7,0] + tmp_get_sample_node4[2,0] = 0
 tmpGetSampleNode4_node3_tech0: - x[3,1,0] - x[3,2,0] - x[3,4,0] - x[3,5,0]
   - x[3,6,0] - x[3,7,0] + tmp_get_sample_node4[3,0] = 0
 tmpGetSampleNode4_node4_tech0: - x[4,1,0] - x[4,2,0] - x[4,3,0] - x[4,5,0]
   - x[4,6,0] - x[4,7,0] + tmp_get_sample_node4[4,0] = 0
 tmpGetSampleNode4_node5_tech0: - x[5,1,0] - x[5,2,0] - x[5,3,0] - x[5,4,0]
   - x[5,6,0] - x[5,7,0] + tmp_get_sample_node4[5,0] = 0
 tmpGetSampleNode4_node6_tech0: - x[6,1,0] - x[6,2,0] - x[6,3,0] - x[6,4,0]
   - x[6,5,0] - x[6,7,0] + tmp_get_sample_node4[6,0] = 0
 tmpGetSampleNode4_node1_tech1: - x[1,2,1] - x[1,3,1] - x[1,4,1] - x[1,5,1]
   - x[1,6,1] - x[1,7,1] + tmp_get_sample_node4[1,1] = 0
 tmpGetSampleNode4_node2_tech1: - x[2,1,1] - x[2,3,1] - x[2,4,1] - x[2,5,1]
   - x[2,6,1] - x[2,7,1] + tmp_get_sample_node4[2,1] = 0
 tmpGetSampleNode4_node3_tech1: - x[3,1,1] - x[3,2,1] - x[3,4,1] - x[3,5,1]
   - x[3,6,1] - x[3,7,1] + tmp_get_sample_node4[3,1] = 0
 tmpGetSampleNode4_node4_tech1: - x[4,1,1] - x[4,2,1] - x[4,3,1] - x[4,5,1]
   - x[4,6,1] - x[4,7,1] + tmp_get_sample_node4[4,1] = 0
 tmpGetSampleNode4_node5_tech1: - x[5,1,1] - x[5,2,1] - x[5,3,1] - x[5,4,1]
   - x[5,6,1] - x[5,7,1] + tmp_get_sample_node4[5,1] = 0
 tmpGetSampleNode4_node6_tech1: - x[6,1,1] - x[6,2,1] - x[6,3,1] - x[6,4,1]
   - x[6,5,1] - x[6,7,1] + tmp_get_sample_node4[6,1] = 0
 tmpSampleBroughtByTechnican1_node1_tech0:
   tmp_sample_brought_by_technican1[1,0] = 0
 tmpSampleBroughtByTechnican1_node1_tech1:
   tmp_sample_brought_by_technican1[1,1] = 0
 tmpSampleBroughtByTechnican1_node2_tech0:
   tmp_sample_brought_by_technican1[2,0] = 0
 tmpSampleBroughtByTechnican1_node2_tech1:
   tmp_sample_brought_by_technican1[2,1] = 0
 tmpSampleBroughtByTechnican1_node3_tech0:
   tmp_sample_brought_by_technican1[3,0] = 0
 tmpSampleBroughtByTechnican1_node3_tech1:
   tmp_sample_brought_by_technican1[3,1] = 0
 tmpSampleBroughtByTechnican1_node4_tech0:
   tmp_sample_brought_by_technican1[4,0] = 0
 tmpSampleBroughtByTechnican1_node4_tech1:
   tmp_sample_brought_by_technican1[4,1] = 0
 tmpSampleBroughtByTechnican1_node5_tech0:
   tmp_sample_brought_by_technican1[5,0] = 0
 tmpSampleBroughtByTechnican1_node5_tech1:
   tmp_sample_brought_by_technican1[5,1] = 0
 tmpSampleBroughtByTechnican1_node6_tech0:
   tmp_sample_brought_by_technican1[6,0] = 0
 tmpSampleBroughtByTechnican1_node6_tech1:
   tmp_sample_brought_by_technican1[6,1] = 0
 tmpSampleBroughtByTechnican2_node1_tech0: - x[1,2,0] - x[1,3,0] - x[1,4,0]
   - x[1,5,0] - x[1,6,0] - x[1,7,0] + tmp_sample_brought_by_technican2[1,0]
   = 0
 tmpSampleBroughtByTechnican2_node1_tech1: - x[1,2,1] - x[1,3,1] - x[1,4,1]
   - x[1,5,1] - x[1,6,1] - x[1,7,1] + tmp_sample_brought_by_technican2[1,1]
   = 0
 tmpSampleBroughtByTechnican2_node2_tech0: - x[2,1,0] - x[2,3,0] - x[2,4,0]
   - x[2,5,0] - x[2,6,0] - x[2,7,0] + tmp_sample_brought_by_technican2[2,0]
   = 0
 tmpSampleBroughtByTechnican2_node2_tech1: - x[2,1,1] - x[2,3,1] - x[2,4,1]
   - x[2,5,1] - x[2,6,1] - x[2,7,1] + tmp_sample_brought_by_technican2[2,1]
   = 0
 tmpSampleBroughtByTechnican2_node3_tech0: - x[3,1,0] - x[3,2,0] - x[3,4,0]
   - x[3,5,0] - x[3,6,0] - x[3,7,0] + tmp_sample_brought_by_technican2[3,0]
   = 0
 tmpSampleBroughtByTechnican2_node3_tech1: - x[3,1,1] - x[3,2,1] - x[3,4,1]
   - x[3,5,1] - x[3,6,1] - x[3,7,1] + tmp_sample_brought_by_technican2[3,1]
   = 0
 tmpSampleBroughtByTechnican2_node4_tech0: - x[4,1,0] - x[4,2,0] - x[4,3,0]
   - x[4,5,0] - x[4,6,0] - x[4,7,0] + tmp_sample_brought_by_technican2[4,0]
   = 0
 tmpSampleBroughtByTechnican2_node4_tech1: - x[4,1,1] - x[4,2,1] - x[4,3,1]
   - x[4,5,1] - x[4,6,1] - x[4,7,1] + tmp_sample_brought_by_technican2[4,1]
   = 0
 tmpSampleBroughtByTechnican2_node5_tech0: - x[5,1,0] - x[5,2,0] - x[5,3,0]
   - x[5,4,0] - x[5,6,0] - x[5,7,0] + tmp_sample_brought_by_technican2[5,0]
   = 0
 tmpSampleBroughtByTechnican2_node5_tech1: - x[5,1,1] - x[5,2,1] - x[5,3,1]
   - x[5,4,1] - x[5,6,1] - x[5,7,1] + tmp_sample_brought_by_technican2[5,1]
   = 0
 tmpSampleBroughtByTechnican2_node6_tech0: - x[6,1,0] - x[6,2,0] - x[6,3,0]
   - x[6,4,0] - x[6,5,0] - x[6,7,0] + tmp_sample_brought_by_technican2[6,0]
   = 0
 tmpSampleBroughtByTechnican2_node6_tech1: - x[6,1,1] - x[6,2,1] - x[6,3,1]
   - x[6,4,1] - x[6,5,1] - x[6,7,1] + tmp_sample_brought_by_technican2[6,1]
   = 0
 maxTime_tech0: B_a[0] <= 60
 maxTime_tech1: B_a[1] <= 60
 tmpMaxTime1_node1: tmp_max_time[1] <= 0
 tmpMaxTime2_node1: tmp_max_time[1] >= 0
 tmpMaxTime1_node2: tmp_max_time[2] <= 0
 tmpMaxTime2_node2: tmp_max_time[2] >= 0
 tmpMaxTime1_node3: tmp_max_time[3] <= 0
 tmpMaxTime2_node3: tmp_max_time[3] >= 0
 tmpMaxTime1_node4: tmp_max_time[4] <= 0
 tmpMaxTime2_node4: tmp_max_time[4] >= 0
 tmpMaxTime1_node5: tmp_max_time[5] <= 0
 tmpMaxTime2_node5: tmp_max_time[5] >= 0
 tmpMaxTime1_node6: tmp_max_time[6] <= 0
 tmpMaxTime2_node6: tmp_max_time[6] >= 0
 completeTimeTech_node1_tech0: x[1,7,0] = 1 -> - t[1] + B[0]
   = 2.135952216253763
 completeTimeTech_node1_tech1: x[1,7,1] = 1 -> - t[1] + B[1]
   = 2.135952216253763
 completeTimeTech_node2_tech0: x[2,7,0] = 1 -> - t[2] + B[0]
   = 5.864070579002398
 completeTimeTech_node2_tech1: x[2,7,1] = 1 -> - t[2] + B[1]
   = 5.864070579002398
 completeTimeTech_node3_tech0: x[3,7,0] = 1 -> - t[3] + B[0]
   = 5.390682740551775
 completeTimeTech_node3_tech1: x[3,7,1] = 1 -> - t[3] + B[1]
   = 5.390682740551775
 completeTimeTech_node4_tech0: x[4,7,0] = 1 -> - t[4] + B[0]
   = 1.776250030653076
 completeTimeTech_node4_tech1: x[4,7,1] = 1 -> - t[4] + B[1]
   = 1.776250030653076
 completeTimeTech_node5_tech0: x[5,7,0] = 1 -> - t[5] + B[0]
   = 3.264367030467128
 completeTimeTech_node5_tech1: x[5,7,1] = 1 -> - t[5] + B[1]
   = 3.264367030467128
 completeTimeTech_node6_tech0: x[6,7,0] = 1 -> - t[6] + B[0]
   = 3.122174194784624
 completeTimeTech_node6_tech1: x[6,7,1] = 1 -> - t[6] + B[1]
   = 3.122174194784624
 completeTimeTech_node0_tech0: x[0,7,0] = 1 -> - t[0] + B[0] = 0
 completeTimeTech_node0_tech1: x[0,7,1] = 1 -> - t[0] + B[1] = 0
 timeAtCusTech1_node1_node2: tmp_time_at_cus_tech1[1,2] = 1 -> t[1] - t[2]
   >= 4.037002486108159
 timeAtCusTech1_node1_node3: tmp_time_at_cus_tech1[1,3] = 1 -> t[1] - t[3]
   >= 4.910943940985136
 timeAtCusTech1_node1_node4: tmp_time_at_cus_tech1[1,4] = 1 -> t[1] - t[4]
   >= 3.856313325757607
 timeAtCusTech1_node1_node5: tmp_time_at_cus_tech1[1,5] = 1 -> t[1] - t[5]
   >= 2.268028153543655
 timeAtCusTech1_node1_node6: tmp_time_at_cus_tech1[1,6] = 1 -> t[1] - t[6]
   >= 3.42049259952323
 timeAtCusTech1_node1_node0: tmp_time_at_cus_tech1[1,0] = 1 -> t[1] - t[0]
   >= 2.135952216253763
 timeAtCusTech1_node2_node1: tmp_time_at_cus_tech1[2,1] = 1 -> - t[1]
   + t[2] >= 4.037002486108159
 timeAtCusTech1_node2_node3: tmp_time_at_cus_tech1[2,3] = 1 -> t[2] - t[3]
   >= 8.027517160888197
 timeAtCusTech1_node2_node4: tmp_time_at_cus_tech1[2,4] = 1 -> t[2] - t[4]
   >= 7.234380632618068
 timeAtCusTech1_node2_node5: tmp_time_at_cus_tech1[2,5] = 1 -> t[2] - t[5]
   >= 2.954634444201998
 timeAtCusTech1_node2_node6: tmp_time_at_cus_tech1[2,6] = 1 -> t[2] - t[6]
   >= 7.279262056747435
 timeAtCusTech1_node2_node0: tmp_time_at_cus_tech1[2,0] = 1 -> t[2] - t[0]
   >= 5.864070579002398
 timeAtCusTech1_node3_node1: tmp_time_at_cus_tech1[3,1] = 1 -> - t[1]
   + t[3] >= 4.910943940985136
 timeAtCusTech1_node3_node2: tmp_time_at_cus_tech1[3,2] = 1 -> - t[2]
   + t[3] >= 8.027517160888197
 timeAtCusTech1_node3_node4: tmp_time_at_cus_tech1[3,4] = 1 -> t[3] - t[4]
   >= 6.744025543671353
 timeAtCusTech1_node3_node5: tmp_time_at_cus_tech1[3,5] = 1 -> t[3] - t[5]
   >= 7.168484058289546
 timeAtCusTech1_node3_node6: tmp_time_at_cus_tech1[3,6] = 1 -> t[3] - t[6]
   >= 2.453704741702595
 timeAtCusTech1_node3_node0: tmp_time_at_cus_tech1[3,0] = 1 -> t[3] - t[0]
   >= 5.390682740551775
 timeAtCusTech1_node4_node1: tmp_time_at_cus_tech1[4,1] = 1 -> - t[1]
   + t[4] >= 3.856313325757607
 timeAtCusTech1_node4_node2: tmp_time_at_cus_tech1[4,2] = 1 -> - t[2]
   + t[4] >= 7.234380632618068
 timeAtCusTech1_node4_node3: tmp_time_at_cus_tech1[4,3] = 1 -> - t[3]
   + t[4] >= 6.744025543671353
 timeAtCusTech1_node4_node5: tmp_time_at_cus_tech1[4,5] = 1 -> t[4] - t[5]
   >= 4.363052970788421
 timeAtCusTech1_node4_node6: tmp_time_at_cus_tech1[4,6] = 1 -> t[4] - t[6]
   >= 4.310013914269966
 timeAtCusTech1_node4_node0: tmp_time_at_cus_tech1[4,0] = 1 -> t[4] - t[0]
   >= 1.776250030653076
 timeAtCusTech1_node5_node1: tmp_time_at_cus_tech1[5,1] = 1 -> - t[1]
   + t[5] >= 2.268028153543655
 timeAtCusTech1_node5_node2: tmp_time_at_cus_tech1[5,2] = 1 -> - t[2]
   + t[5] >= 2.954634444201998
 timeAtCusTech1_node5_node3: tmp_time_at_cus_tech1[5,3] = 1 -> - t[3]
   + t[5] >= 7.168484058289546
 timeAtCusTech1_node5_node4: tmp_time_at_cus_tech1[5,4] = 1 -> - t[4]
   + t[5] >= 4.363052970788421
 timeAtCusTech1_node5_node6: tmp_time_at_cus_tech1[5,6] = 1 -> t[5] - t[6]
   >= 5.598514571471891
 timeAtCusTech1_node5_node0: tmp_time_at_cus_tech1[5,0] = 1 -> t[5] - t[0]
   >= 3.264367030467128
 timeAtCusTech1_node6_node1: tmp_time_at_cus_tech1[6,1] = 1 -> - t[1]
   + t[6] >= 3.42049259952323
 timeAtCusTech1_node6_node2: tmp_time_at_cus_tech1[6,2] = 1 -> - t[2]
   + t[6] >= 7.279262056747435
 timeAtCusTech1_node6_node3: tmp_time_at_cus_tech1[6,3] = 1 -> - t[3]
   + t[6] >= 2.453704741702595
 timeAtCusTech1_node6_node4: tmp_time_at_cus_tech1[6,4] = 1 -> - t[4]
   + t[6] >= 4.310013914269966
 timeAtCusTech1_node6_node5: tmp_time_at_cus_tech1[6,5] = 1 -> - t[5]
   + t[6] >= 5.598514571471891
 timeAtCusTech1_node6_node0: tmp_time_at_cus_tech1[6,0] = 1 -> t[6] - t[0]
   >= 3.122174194784624
 timeGetSample_node1_node2: tmp_time_at_cus_tech1[2,1] = 1 -> - t[1] + T[2]
   = 4.037002486108159
 timeGetSample_node1_node3: tmp_time_at_cus_tech1[3,1] = 1 -> - t[1] + T[3]
   = 4.910943940985136
 timeGetSample_node1_node4: tmp_time_at_cus_tech1[4,1] = 1 -> - t[1] + T[4]
   = 3.856313325757607
 timeGetSample_node1_node5: tmp_time_at_cus_tech1[5,1] = 1 -> - t[1] + T[5]
   = 2.268028153543655
 timeGetSample_node1_node6: tmp_time_at_cus_tech1[6,1] = 1 -> - t[1] + T[6]
   = 3.42049259952323
 timeGetSample_node2_node1: tmp_time_at_cus_tech1[1,2] = 1 -> T[1] - t[2]
   = 4.037002486108159
 timeGetSample_node2_node3: tmp_time_at_cus_tech1[3,2] = 1 -> - t[2] + T[3]
   = 8.027517160888197
 timeGetSample_node2_node4: tmp_time_at_cus_tech1[4,2] = 1 -> - t[2] + T[4]
   = 7.234380632618068
 timeGetSample_node2_node5: tmp_time_at_cus_tech1[5,2] = 1 -> - t[2] + T[5]
   = 2.954634444201998
 timeGetSample_node2_node6: tmp_time_at_cus_tech1[6,2] = 1 -> - t[2] + T[6]
   = 7.279262056747435
 timeGetSample_node3_node1: tmp_time_at_cus_tech1[1,3] = 1 -> T[1] - t[3]
   = 4.910943940985136
 timeGetSample_node3_node2: tmp_time_at_cus_tech1[2,3] = 1 -> T[2] - t[3]
   = 8.027517160888197
 timeGetSample_node3_node4: tmp_time_at_cus_tech1[4,3] = 1 -> - t[3] + T[4]
   = 6.744025543671353
 timeGetSample_node3_node5: tmp_time_at_cus_tech1[5,3] = 1 -> - t[3] + T[5]
   = 7.168484058289546
 timeGetSample_node3_node6: tmp_time_at_cus_tech1[6,3] = 1 -> - t[3] + T[6]
   = 2.453704741702595
 timeGetSample_node4_node1: tmp_time_at_cus_tech1[1,4] = 1 -> T[1] - t[4]
   = 3.856313325757607
 timeGetSample_node4_node2: tmp_time_at_cus_tech1[2,4] = 1 -> T[2] - t[4]
   = 7.234380632618068
 timeGetSample_node4_node3: tmp_time_at_cus_tech1[3,4] = 1 -> T[3] - t[4]
   = 6.744025543671353
 timeGetSample_node4_node5: tmp_time_at_cus_tech1[5,4] = 1 -> - t[4] + T[5]
   = 4.363052970788421
 timeGetSample_node4_node6: tmp_time_at_cus_tech1[6,4] = 1 -> - t[4] + T[6]
   = 4.310013914269966
 timeGetSample_node5_node1: tmp_time_at_cus_tech1[1,5] = 1 -> T[1] - t[5]
   = 2.268028153543655
 timeGetSample_node5_node2: tmp_time_at_cus_tech1[2,5] = 1 -> T[2] - t[5]
   = 2.954634444201998
 timeGetSample_node5_node3: tmp_time_at_cus_tech1[3,5] = 1 -> T[3] - t[5]
   = 7.168484058289546
 timeGetSample_node5_node4: tmp_time_at_cus_tech1[4,5] = 1 -> T[4] - t[5]
   = 4.363052970788421
 timeGetSample_node5_node6: tmp_time_at_cus_tech1[6,5] = 1 -> - t[5] + T[6]
   = 5.598514571471891
 timeGetSample_node6_node1: tmp_time_at_cus_tech1[1,6] = 1 -> T[1] - t[6]
   = 3.42049259952323
 timeGetSample_node6_node2: tmp_time_at_cus_tech1[2,6] = 1 -> T[2] - t[6]
   = 7.279262056747435
 timeGetSample_node6_node3: tmp_time_at_cus_tech1[3,6] = 1 -> T[3] - t[6]
   = 2.453704741702595
 timeGetSample_node6_node4: tmp_time_at_cus_tech1[4,6] = 1 -> T[4] - t[6]
   = 4.310013914269966
 timeGetSample_node6_node5: tmp_time_at_cus_tech1[5,6] = 1 -> T[5] - t[6]
   = 5.598514571471891
 timeGetSample_node0_node1: tmp_time_at_cus_tech1[1,0] = 1 -> T[1] - t[0]
   = 2.135952216253763
 timeGetSample_node0_node2: tmp_time_at_cus_tech1[2,0] = 1 -> T[2] - t[0]
   = 5.864070579002398
 timeGetSample_node0_node3: tmp_time_at_cus_tech1[3,0] = 1 -> T[3] - t[0]
   = 5.390682740551775
 timeGetSample_node0_node4: tmp_time_at_cus_tech1[4,0] = 1 -> T[4] - t[0]
   = 1.776250030653076
 timeGetSample_node0_node5: tmp_time_at_cus_tech1[5,0] = 1 -> T[5] - t[0]
   = 3.264367030467128
 timeGetSample_node0_node6: tmp_time_at_cus_tech1[6,0] = 1 -> T[6] - t[0]
   = 3.122174194784624
 sampleBroughtByTechnican1_node1tech0: tmp_sample_brought_by_technican1[1,0] = 1 ->
   D[1,0] = 0
 sampleBroughtByTechnican1_node1tech1: tmp_sample_brought_by_technican1[1,1] = 1 ->
   D[1,1] = 0
 sampleBroughtByTechnican1_node2tech0: tmp_sample_brought_by_technican1[2,0] = 1 ->
   D[2,0] = 0
 sampleBroughtByTechnican1_node2tech1: tmp_sample_brought_by_technican1[2,1] = 1 ->
   D[2,1] = 0
 sampleBroughtByTechnican1_node3tech0: tmp_sample_brought_by_technican1[3,0] = 1 ->
   D[3,0] = 0
 sampleBroughtByTechnican1_node3tech1: tmp_sample_brought_by_technican1[3,1] = 1 ->
   D[3,1] = 0
 sampleBroughtByTechnican1_node4tech0: tmp_sample_brought_by_technican1[4,0] = 1 ->
   D[4,0] = 0
 sampleBroughtByTechnican1_node4tech1: tmp_sample_brought_by_technican1[4,1] = 1 ->
   D[4,1] = 0
 sampleBroughtByTechnican1_node5tech0: tmp_sample_brought_by_technican1[5,0] = 1 ->
   D[5,0] = 0
 sampleBroughtByTechnican1_node5tech1: tmp_sample_brought_by_technican1[5,1] = 1 ->
   D[5,1] = 0
 sampleBroughtByTechnican1_node6tech0: tmp_sample_brought_by_technican1[6,0] = 1 ->
   D[6,0] = 0
 sampleBroughtByTechnican1_node6tech1: tmp_sample_brought_by_technican1[6,1] = 1 ->
   D[6,1] = 0
 sampleBroughtByTechnican2_node1tech0: tmp_sample_brought_by_technican2[1,0] = 1 ->
   D[1,0] + T[1] - B[0] = 0
 sampleBroughtByTechnican2_node1tech1: tmp_sample_brought_by_technican2[1,1] = 1 ->
   D[1,1] + T[1] - B[1] = 0
 sampleBroughtByTechnican2_node2tech0: tmp_sample_brought_by_technican2[2,0] = 1 ->
   D[2,0] + T[2] - B[0] = 0
 sampleBroughtByTechnican2_node2tech1: tmp_sample_brought_by_technican2[2,1] = 1 ->
   D[2,1] + T[2] - B[1] = 0
 sampleBroughtByTechnican2_node3tech0: tmp_sample_brought_by_technican2[3,0] = 1 ->
   D[3,0] + T[3] - B[0] = 0
 sampleBroughtByTechnican2_node3tech1: tmp_sample_brought_by_technican2[3,1] = 1 ->
   D[3,1] + T[3] - B[1] = 0
 sampleBroughtByTechnican2_node4tech0: tmp_sample_brought_by_technican2[4,0] = 1 ->
   D[4,0] + T[4] - B[0] = 0
 sampleBroughtByTechnican2_node4tech1: tmp_sample_brought_by_technican2[4,1] = 1 ->
   D[4,1] + T[4] - B[1] = 0
 sampleBroughtByTechnican2_node5tech0: tmp_sample_brought_by_technican2[5,0] = 1 ->
   D[5,0] + T[5] - B[0] = 0
 sampleBroughtByTechnican2_node5tech1: tmp_sample_brought_by_technican2[5,1] = 1 ->
   D[5,1] + T[5] - B[1] = 0
 sampleBroughtByTechnican2_node6tech0: tmp_sample_brought_by_technican2[6,0] = 1 ->
   D[6,0] + T[6] - B[0] = 0
 sampleBroughtByTechnican2_node6tech1: tmp_sample_brought_by_technican2[6,1] = 1 ->
   D[6,1] + T[6] - B[1] = 0
 sampleTechTemp1_tech0: tmp_sample_tech_temp[0] = 1 -> - B[0] + B_a[0] = 0
 sampleTechTemp1_tech1: tmp_sample_tech_temp[1] = 1 -> - B[1] + B_a[1] = 0
 maxTime_node1: tmp_max_time[1] = 1 -> t_a[1] <= 58.50740688502749
 maxTime_node2: tmp_max_time[2] = 1 -> t_a[2] <= 55.90221573997422
 maxTime_node3: tmp_max_time[3] = 1 -> t_a[3] <= 56.23301688009635
 maxTime_node4: tmp_max_time[4] = 1 -> t_a[4] <= 58.75876503882074
 maxTime_node5: tmp_max_time[5] = 1 -> t_a[5] <= 57.71887605099887
 maxTime_node6: tmp_max_time[6] = 1 -> t_a[6] <= 57.81823971930713
Bounds
 D[1,0] <= 60
 D[2,0] <= 60
 D[3,0] <= 60
 D[4,0] <= 60
 D[5,0] <= 60
 D[6,0] <= 60
 D[1,1] <= 60
 D[2,1] <= 60
 D[3,1] <= 60
 D[4,1] <= 60
 D[5,1] <= 60
 D[6,1] <= 60
 t[1] <= 60
 t_a[1] <= 60
 T[1] <= 60
 t[2] <= 60
 t_a[2] <= 60
 T[2] <= 60
 t[3] <= 60
 t_a[3] <= 60
 T[3] <= 60
 t[4] <= 60
 t_a[4] <= 60
 T[4] <= 60
 t[5] <= 60
 t_a[5] <= 60
 T[5] <= 60
 t[6] <= 60
 t_a[6] <= 60
 T[6] <= 60
 t[7] <= 60
 t_a[7] <= 60
 T[7] <= 60
 t[0] <= 60
 t_a[0] <= 60
 T[0] <= 60
 B[0] <= 60
 B_a[0] <= 60
 B[1] <= 60
 B_a[1] <= 60
Binaries
 x[2,1,0] x[3,1,0] x[4,1,0] x[5,1,0] x[6,1,0] x[0,1,0] x[1,2,0] x[3,2,0]
 x[4,2,0] x[5,2,0] x[6,2,0] x[0,2,0] x[1,3,0] x[2,3,0] x[4,3,0] x[5,3,0]
 x[6,3,0] x[0,3,0] x[1,4,0] x[2,4,0] x[3,4,0] x[5,4,0] x[6,4,0] x[0,4,0]
 x[1,5,0] x[2,5,0] x[3,5,0] x[4,5,0] x[6,5,0] x[0,5,0] x[1,6,0] x[2,6,0]
 x[3,6,0] x[4,6,0] x[5,6,0] x[0,6,0] x[1,7,0] x[2,7,0] x[3,7,0] x[4,7,0]
 x[5,7,0] x[6,7,0] x[0,7,0] x[2,1,1] x[3,1,1] x[4,1,1] x[5,1,1] x[6,1,1]
 x[0,1,1] x[1,2,1] x[3,2,1] x[4,2,1] x[5,2,1] x[6,2,1] x[0,2,1] x[1,3,1]
 x[2,3,1] x[4,3,1] x[5,3,1] x[6,3,1] x[0,3,1] x[1,4,1] x[2,4,1] x[3,4,1]
 x[5,4,1] x[6,4,1] x[0,4,1] x[1,5,1] x[2,5,1] x[3,5,1] x[4,5,1] x[6,5,1]
 x[0,5,1] x[1,6,1] x[2,6,1] x[3,6,1] x[4,6,1] x[5,6,1] x[0,6,1] x[1,7,1]
 x[2,7,1] x[3,7,1] x[4,7,1] x[5,7,1] x[6,7,1] x[0,7,1]
Generals
 tmp_time_at_cus_tech1[1,2] tmp_time_at_cus_tech1[1,3]
 tmp_time_at_cus_tech1[1,4] tmp_time_at_cus_tech1[1,5]
 tmp_time_at_cus_tech1[1,6] tmp_time_at_cus_tech1[1,0]
 tmp_time_at_cus_tech1[2,1] tmp_time_at_cus_tech1[2,3]
 tmp_time_at_cus_tech1[2,4] tmp_time_at_cus_tech1[2,5]
 tmp_time_at_cus_tech1[2,6] tmp_time_at_cus_tech1[2,0]
 tmp_time_at_cus_tech1[3,1] tmp_time_at_cus_tech1[3,2]
 tmp_time_at_cus_tech1[3,4] tmp_time_at_cus_tech1[3,5]
 tmp_time_at_cus_tech1[3,6] tmp_time_at_cus_tech1[3,0]
 tmp_time_at_cus_tech1[4,1] tmp_time_at_cus_tech1[4,2]
 tmp_time_at_cus_tech1[4,3] tmp_time_at_cus_tech1[4,5]
 tmp_time_at_cus_tech1[4,6] tmp_time_at_cus_tech1[4,0]
 tmp_time_at_cus_tech1[5,1] tmp_time_at_cus_tech1[5,2]
 tmp_time_at_cus_tech1[5,3] tmp_time_at_cus_tech1[5,4]
 tmp_time_at_cus_tech1[5,6] tmp_time_at_cus_tech1[5,0]
 tmp_time_at_cus_tech1[6,1] tmp_time_at_cus_tech1[6,2]
 tmp_time_at_cus_tech1[6,3] tmp_time_at_cus_tech1[6,4]
 tmp_time_at_cus_tech1[6,5] tmp_time_at_cus_tech1[6,0]
 tmp_get_sample_node4[1,0] tmp_get_sample_node4[2,0]
 tmp_get_sample_node4[3,0] tmp_get_sample_node4[4,0]
 tmp_get_sample_node4[5,0] tmp_get_sample_node4[6,0]
 tmp_get_sample_node4[1,1] tmp_get_sample_node4[2,1]
 tmp_get_sample_node4[3,1] tmp_get_sample_node4[4,1]
 tmp_get_sample_node4[5,1] tmp_get_sample_node4[6,1]
 tmp_sample_brought_by_technican1[1,0]
 tmp_sample_brought_by_technican1[1,1]
 tmp_sample_brought_by_technican1[2,0]
 tmp_sample_brought_by_technican1[2,1]
 tmp_sample_brought_by_technican1[3,0]
 tmp_sample_brought_by_technican1[3,1]
 tmp_sample_brought_by_technican1[4,0]
 tmp_sample_brought_by_technican1[4,1]
 tmp_sample_brought_by_technican1[5,0]
 tmp_sample_brought_by_technican1[5,1]
 tmp_sample_brought_by_technican1[6,0]
 tmp_sample_brought_by_technican1[6,1]
 tmp_sample_brought_by_technican2[1,0]
 tmp_sample_brought_by_technican2[1,1]
 tmp_sample_brought_by_technican2[2,0]
 tmp_sample_brought_by_technican2[2,1]
 tmp_sample_brought_by_technican2[3,0]
 tmp_sample_brought_by_technican2[3,1]
 tmp_sample_brought_by_technican2[4,0]
 tmp_sample_brought_by_technican2[4,1]
 tmp_sample_brought_by_technican2[5,0]
 tmp_sample_brought_by_technican2[5,1]
 tmp_sample_brought_by_technican2[6,0]
 tmp_sample_brought_by_technican2[6,1] tmp_sample_tech_temp[0]
 tmp_sample_tech_temp[1] tmp_max_time[1] tmp_max_time[2] tmp_max_time[3]
 tmp_max_time[4] tmp_max_time[5] tmp_max_time[6]
General Constraints
 tmpSampleTechTemp1_tech0: tmp_sample_tech_temp[0] = OR (
   tmp_sample_brought_by_technican2[1,0] ,
   tmp_sample_brought_by_technican2[2,0] ,
   tmp_sample_brought_by_technican2[3,0] ,
   tmp_sample_brought_by_technican2[4,0] ,
   tmp_sample_brought_by_technican2[5,0] ,
   tmp_sample_brought_by_technican2[6,0] )
 tmpSampleTechTemp1_tech1: tmp_sample_tech_temp[1] = OR (
   tmp_sample_brought_by_technican2[1,1] ,
   tmp_sample_brought_by_technican2[2,1] ,
   tmp_sample_brought_by_technican2[3,1] ,
   tmp_sample_brought_by_technican2[4,1] ,
   tmp_sample_brought_by_technican2[5,1] ,
   tmp_sample_brought_by_technican2[6,1] )
End

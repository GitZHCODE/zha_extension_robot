
 MODULE Module1 

 !Flag, end the program 
 
 VAR bool bProgEnd; 
 
 !Constant for the joint calibrate position 
 
 CONST jointtarget calib_pos := [[-25, 15, 0, 0, 90.0, 180], [0, 9E9, 9E9, 9E9, 9E9, 9E9]]; 

 PERS tooldata zHWC := [TRUE, [[0,0,994],[1, 0, 0, 0]], [5, [0, 0, 1],[1.000000,0.000000,0.000000,0.000000],0,0,0]]; 

 PROC Main() 
 
 Init; 
 
 mv_Calib; 
 
 mv_Custom; 
 
 mv_Calib; 
 
 ENDPROC 
 
 PROC Init() 
 
 !Defined setting of the variables 

 bProgEnd := FALSE; 
 
 ENDPROC 
 
 PROC mv_Calib() 
 
 ENDPROC 
 
 PROC mv_Custom() 
 
  CONST num count := 54;
  CONST robtarget poses {count} := 
 [ 
  [[1800.000000,-250.000000,0.000000], [0.000000,1.000000,0.000000,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1643.135742,-1117.394775,0.000000], [-0.000000,1.000000,-0.000000,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1643.135742,-1117.394775,-800.000000], [-0.000000,1.000000,-0.000000,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1700.000000,-1117.394775,-800.000000], [-0.000000,1.000000,-0.000000,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1708.179688,-1133.131836,-758.052856], [-0.000000,0.997441,0.071493,-0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1716.042114,-1148.347412,-716.105652], [-0.000000,0.989039,0.147657,-0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1723.269775,-1162.519775,-674.158508], [-0.000000,0.975123,0.221667,-0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1729.545776,-1175.127686,-632.211304], [-0.000000,0.958042,0.286626,-0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1734.552368,-1185.649170,-590.264160], [-0.000000,0.941219,0.337796,-0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1738.607178,-1194.605835,-548.316956], [-0.000000,0.925801,0.378011,-0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1742.027344,-1202.519531,-506.369843], [-0.000000,0.911837,0.410553,-0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1745.130127,-1209.911743,-464.422638], [-0.000000,0.898768,0.438424,-0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1660.993530,-1317.628052,-464.422638], [-0.000000,0.898768,0.438424,-0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1660.993530,-1317.628052,0.000000], [-0.000000,0.898768,0.438424,-0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1991.653564,-1117.394775,0.000000], [0.000000,1.000000,0.000000,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1991.653564,-1117.394775,-800.000000], [0.000000,1.000000,0.000000,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1934.789551,-1117.394775,-800.000000], [0.000000,1.000000,0.000000,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1926.609985,-1101.657593,-758.052856], [0.000000,0.997441,0.071493,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1918.747437,-1086.442261,-716.105652], [0.000000,0.989039,0.147657,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1911.519775,-1072.269653,-674.158508], [0.000000,0.975123,0.221666,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1905.243896,-1059.661987,-632.211304], [0.000000,0.958042,0.286626,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1900.237061,-1049.140625,-590.264160], [0.000000,0.941219,0.337796,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1896.182251,-1040.183716,-548.316956], [0.000000,0.925801,0.378011,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1892.762207,-1032.269897,-506.369843], [0.000000,0.911837,0.410553,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1889.659302,-1024.877930,-464.422638], [0.000000,0.898768,0.438424,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1973.796021,-917.161682,-464.422638], [0.000000,0.898768,0.438424,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1973.796021,-917.161682,0.000000], [0.000000,0.898768,0.438424,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1817.394775,-1291.653687,0.000000], [0.000000,0.707107,-0.707107,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1817.394775,-1291.653687,-800.000000], [0.000000,0.707107,-0.707107,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1817.394775,-1234.789551,-800.000000], [0.000000,0.707107,-0.707107,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1833.131958,-1226.609985,-758.052856], [-0.000000,0.755850,-0.654744,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1848.347412,-1218.747437,-716.105652], [-0.000000,0.803766,-0.594946,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1862.519897,-1211.519775,-674.158508], [-0.000000,0.846258,-0.532774,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1875.127563,-1205.243774,-632.211304], [-0.000000,0.880114,-0.474763,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1885.648926,-1200.237183,-590.264160], [-0.000000,0.904400,-0.426685,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1894.605835,-1196.182373,-548.316956], [-0.000000,0.921934,-0.387346,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1902.519409,-1192.762085,-506.369843], [-0.000000,0.935070,-0.354462,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1909.911621,-1189.659302,-464.422638], [-0.000000,0.945538,-0.325512,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[2017.627686,-1273.796021,-464.422638], [-0.000000,0.945538,-0.325512,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[2017.627686,-1273.796021,0.000000], [-0.000000,0.945538,-0.325512,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1817.394775,-943.135986,0.000000], [-0.000000,0.707107,-0.707107,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1817.394775,-943.135986,-800.000000], [-0.000000,0.707107,-0.707107,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1817.394775,-1000.000000,-800.000000], [-0.000000,0.707107,-0.707107,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1801.657715,-1008.179688,-758.052856], [-0.000000,0.755850,-0.654745,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1786.442261,-1016.042114,-716.105652], [-0.000000,0.803766,-0.594946,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1772.269775,-1023.269897,-674.158508], [-0.000000,0.846257,-0.532774,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1759.661987,-1029.545776,-632.211304], [-0.000000,0.880114,-0.474763,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1749.140503,-1034.552490,-590.264160], [-0.000000,0.904400,-0.426685,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1740.183594,-1038.607178,-548.316956], [-0.000000,0.921934,-0.387347,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1732.270020,-1042.027466,-506.369843], [-0.000000,0.935071,-0.354461,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1724.877808,-1045.130249,-464.422638], [-0.000000,0.945538,-0.325512,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1617.161499,-960.993469,-464.422638], [-0.000000,0.945538,-0.325512,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1617.161499,-960.993469,0.000000], [-0.000000,0.945538,-0.325512,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] ,
  [[1800.000000,-250.000000,0.000000], [0.000000,1.000000,0.000000,0.000000], [0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]] 
 ]; 
  CONST speeddata vels {count} := 
 [ 
 v1000,
 v1000,
 v1000,
 v100,
 v100,
 v100,
 v100,
 v100,
 v100,
 v100,
 v100,
 v100,
 v100,
 v1000,
 v1000,
 v1000,
 v100,
 v100,
 v100,
 v100,
 v100,
 v100,
 v100,
 v100,
 v100,
 v100,
 v1000,
 v1000,
 v1000,
 v100,
 v100,
 v100,
 v100,
 v100,
 v100,
 v100,
 v100,
 v100,
 v100,
 v1000,
 v1000,
 v1000,
 v100,
 v100,
 v100,
 v100,
 v100,
 v100,
 v100,
 v100,
 v100,
 v100,
 v1000,
 v1000
 ]; 
 FOR i FROM 1 TO count DO 
 MoveL poses{i}, vels{i}, z1, zHWC\WObj:=wobj0;
 ENDFOR
 ENDPROC 
 
 ENDMODULE 
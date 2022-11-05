function intergrated_difference = Genetic_Algorithm(c)
intergrated_difference=0;
c1=c(1);
c2=c(2);
%c1 = 0.05*1.1; % 0.055
%c2 = 0.012;  % 0.36
for t = 1:100
    qd_test = inverseKinematics_ga(t,c1,c2);
    qd_right = inverseKinematics_ga(t,0.05*1.1,0.012);
    intergrated_difference=+sum(abs(qd_right-qd_test));
end
end

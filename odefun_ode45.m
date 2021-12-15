function Q = odefun_ode45(mbs, q, t)

Cq = constraints_dq(mbs, q);
Ct = constraints_dt(mbs, t);
qip = -Cq \ Ct; % -Cq^-1*Ct

g = LHS_acc_eq(mbs, q, qip, t);
qipp = Cq \ g;

Q = [qip; qipp];
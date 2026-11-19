library(jsonlite)
d=fromJSON('integration/concept-followup-20261119/concept-1635/objects/distance.json')$observations
d$dv = with(d,v-181.14964-.655*254.391622/tri)
d$du = d$u-330.01302
print(aggregate(cbind(du,dv)~id,d,median))
for (i in c(0,1,3,4)) {
  m=lm(dv~du, subset(d,id!=i)); a=d[d$id==i,]; pred=.655*254.391622/(a$v-181.14964-predict(m,newdata=a)); e=a$nominal-a$tri; efix=pred-a$tri
  print(c(i=i, intercept=unname(coef(m)[1]),slope=unname(coef(m)[2]),median_old_abs=median(abs(e)), median_corrected_abs=median(abs(efix)),p95_old_abs=unname(quantile(abs(e),.95)),p95_new_abs=unname(quantile(abs(efix),.95))))
}

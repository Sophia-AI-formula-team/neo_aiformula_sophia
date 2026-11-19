library(jsonlite)
a <- fromJSON("run-trace.json")
png("run-trace.png",width=1180,height=680,res=110,type="cairo")
par(mfrow=c(2,1),mar=c(2.5,4.1,2.7,1),las=1,mgp=c(2.7,.7,0),bty="l")
mode=a$mode
for(i in 2:length(mode))if(is.na(mode[i]))mode[i]=mode[i-1]
bg=function(y1,y2) {
 for(type in c("auto","manual")){
  selected=which(mode==type)
  rect(a$t[selected],y1,a$t[selected]+.1,y2,col=if(type=="auto") "#dcf0e7" else "#e7eef7",border=NA)
 }
}
plot(range(a$t),c(-4,5),type="n",xlab="",ylab="Speed (m/s)",main="Concept | 16:35:34–16:37:20 JST   ·   Wheel speed and motor request")
bg(-4,5); abline(h=0,col="#adb7b2");lines(a$t,a$wheel,col="#08755e",lwd=1.35);lines(a$t,a$vcmd,col="#aa602d",lwd=1.2,type="s")
legend("bottomleft",ncol=4,legend=c("Wheel mean","Motor request","Auto mode","Manual mode"),lty=c(1,1,NA,NA),pch=c(NA,NA,15,15),lwd=2,col=c("#08755e","#aa602d","#bcdccf","#c1d1e5"),bty="n",cex=.87)
plot(range(a$t),range(c(a$wcmd,a$gyro),na.rm=T),type="n",xlab="Seconds after recording start",ylab="Yaw rate (rad/s)",main="Measured and requested yaw rate   ·   Left turn positive")
bg(-6,6);abline(h=0,col="#adb7b2");lines(a$t,a$gyro,col="#08755e",lwd=1.35);lines(a$t,a$wcmd,col="#aa602d",lwd=1.2,type="s")
legend("bottomleft",ncol=2,legend=c("VectorNav gyro z","Motor yaw request"),lty=1,lwd=2,col=c("#08755e","#aa602d"),bty="n",cex=.87)
dev.off()

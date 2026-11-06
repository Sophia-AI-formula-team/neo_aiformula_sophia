using System;

namespace LapLab {
    // Move last frame's raw four road samples into the present body frame.
    // Reuse one overlapping beam for one camera interval only. The far beam
    // sees new pavement and remains an independent preview of incoming turns.
    public sealed class OdometryLaneGate {
        readonly float[] ahead,limits;
        readonly bool[] held=new bool[4];
        float[] previous;
        float previousTime,previousSpeed,previousYaw;

        public OdometryLaneGate(float[] ahead,float[] limits){
            if(ahead.Length!=4 || limits.Length!=3)throw new ArgumentException("Expected four beams and three cutoffs");
            this.ahead=(float[])ahead.Clone();this.limits=(float[])limits.Clone();
            foreach(float limit in limits)if(!(limit>0) || float.IsInfinity(limit))throw new ArgumentOutOfRangeException("limits");
        }

        public int Apply(float[] line,float time,float speed,float yaw){
            var raw=(float[])line.Clone();
            float dt=time-previousTime;
            int mask=0;
            if(previous!=null && dt>0 && dt<=.30f){
                double turn=.5*(yaw+previousYaw)*dt;
                double travel=.5*(speed+previousSpeed)*dt;
                double c=Math.Cos(turn),s=Math.Sin(turn);
                double forward=travel*(Math.Abs(turn)<1e-6?1-turn*turn/6:s/turn);
                double right=travel*(Math.Abs(turn)<1e-6?.5*turn:(1-c)/turn);
                var z=new double[4];var x=new double[4];bool valid=true;
                for(int i=0;i<4;i++){
                    double f=ahead[i]-forward,r=previous[i]-right;
                    z[i]=c*f+s*r;x[i]=c*r-s*f;
                    if(float.IsNaN(previous[i]) || (i>0 && z[i]-z[i-1]<=.02))valid=false;
                }
                if(valid)for(int i=0;i<limits.Length;i++){
                    if(held[i] || float.IsNaN(raw[i]) || ahead[i]<z[0] || ahead[i]>z[3])continue;
                    int k=0;while(k<2 && z[k+1]<=ahead[i])k++;
                    double predicted=x[k]+(ahead[i]-z[k])*(x[k+1]-x[k])/(z[k+1]-z[k]);
                    if(Math.Abs(raw[i]-predicted)<=limits[i])continue;
                    mask|=1<<i;line[i]=(float)predicted;
                }
            }
            for(int i=0;i<held.Length;i++)held[i]=(mask&(1<<i))!=0;
            previous=raw;previousTime=time;previousSpeed=speed;previousYaw=yaw;
            return mask;
        }
    }
}

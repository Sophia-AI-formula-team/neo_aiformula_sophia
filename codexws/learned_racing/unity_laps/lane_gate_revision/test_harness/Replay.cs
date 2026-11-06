using System;
using System.Globalization;
using System.Linq;
using LapLab;
public static class Replay {
    static float[] Values(string row){return row.Split(' ').Select(x=>float.Parse(x,CultureInfo.InvariantCulture)).ToArray();}
    public static void Main(){
        System.Threading.Thread.CurrentThread.CurrentCulture=CultureInfo.InvariantCulture;
        float[] limits=Values(Console.ReadLine()),ahead={2,3.5f,5.5f,8f};OdometryLaneGate gate=null;
        string row;
        while((row=Console.ReadLine())!=null){
            if(row=="reset"){gate=new OdometryLaneGate(ahead,limits);continue;}
            var a=Values(row);var lane=a.Skip(3).ToArray();int mask=gate.Apply(lane,a[0],a[1],a[2]);
            Console.WriteLine(mask+" "+string.Join(" ",lane.Select(v=>v.ToString("R")).ToArray()));
        }
    }
}

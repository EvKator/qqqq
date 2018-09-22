using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Robot.Common;
using System.Reflection;



namespace BagirovIslyam.RobotChallange
{
    public class BagirovIslyamAlgorithm : IRobotAlgorithm
    {
        public BagirovIslyamAlgorithm()
        {
            paths = new List<Path>();
        }

        string IRobotAlgorithm.Author
        {
            get
            {
                return "Bagirov Islyam";
            }
        }

        string IRobotAlgorithm.Description
        {
            get
            {
                return "Bagirov Algorithm";
            }
        }
        
        private List<Path> paths;
        bool move = true;

        RobotCommand IRobotAlgorithm.DoStep(IList<Robot.Common.Robot> robots, int robotToMoveIndex, Map map)
        {
            //robots[robotToMoveIndex].Energy += 10000;
            Robot.Common.Robot movingRobot = robots[robotToMoveIndex]; if ((movingRobot.Energy > 500) && (robots.Count < map.Stations.Count)) { return new CreateNewRobotCommand(); }


            Position stationPosition = FindNearestFreeStation(robots[robotToMoveIndex], map, robots);


            if (robotToMoveIndex >= paths.Count)
            {
                var path = new Path(movingRobot.Position, stationPosition);
                path = path.DevideWithMinSteps();
                paths.Add(path) ;
            }

            if (stationPosition == null) return null;

            if (map.Stations.FirstOrDefault(s => s.Position == movingRobot.Position) != null) return new CollectEnergyCommand();
            else {
                

                return new MoveCommand() { NewPosition = paths[robotToMoveIndex].MakeStep() };
            }

        }




        public Position FindNearestFreeStation(Robot.Common.Robot movingRobot, Map map, IList<Robot.Common.Robot> robots)
        {
            EnergyStation nearest = null; int minDistance = int.MaxValue; foreach (var station in map.Stations)
            {
                if (paths.FindIndex(p => p.FinPosition == station.Position) != -1)
                    continue;
                if (IsStationFree(station, movingRobot, robots))
                {
                    int d = DistanceHelper.FindDistance(station.Position, movingRobot.Position);

                    if (d < minDistance) { minDistance = d; nearest = station; }
                }
            }

            return nearest == null ? null : nearest.Position;
        }

        public bool IsStationFree(EnergyStation station, Robot.Common.Robot movingRobot, IList<Robot.Common.Robot> robots) { return IsCellFree(station.Position, movingRobot, robots); }


        public bool IsCellFree(Position cell, Robot.Common.Robot movingRobot, IList<Robot.Common.Robot> robots) { foreach (var robot in robots) { if (robot != movingRobot) { if (robot.Position == cell) return false; } } return true; }




    }


    public struct Vector
    {
        public int X { get; set; }
        public int Y { get; set; }

        public Vector(int X, int Y)
        {
            this.X = X;
            this.Y = Y;
        }
        

        public int L
        {
            get
            {
                return (int)Math.Pow(X, 2) + (int)Math.Pow(Y, 2);
            }
        }

        public Vector Reduce(uint l = 1)
        {
            Vector vec = new Vector(X,Y);
            for(int i = 0; i < l; i++)
            {
                if (Math.Abs(vec.X) > Math.Abs(vec.Y))//////////////////////////////
                    vec.X += -1 * Math.Sign(X);
                else
                    vec.Y += -1 * Math.Sign(Y);
            }
            return vec;

        }

    }

    public class Path: ICloneable
    {
        public Position StartPosition { get; set; }
        public Position FinPosition { get; set; }
        List<Vector> vectors;

        public Path(params Vector[] vector)
        {
            this.vectors = new List<Vector>() {  };
            vectors.AddRange(vector);
            this.StartPosition = new Position(0,0);
            if(vector.Length > 0)
            this.FinPosition = StartPosition.Add(new Position(vector[vector.Length - 1].X, vector[vector.Length - 1].Y));
            
        }

        public Path(Position p1, Position p2)
        {
            this.StartPosition = p1;
            this.FinPosition = p2;
            this.vectors = new List<Vector>{
                new Vector(p2.Sub(p1).X, p2.Sub(p1).Y) };
        }

        public object Clone()
        {
            return new Path(vectors.ToArray()) { StartPosition = this.StartPosition, FinPosition = this.FinPosition } ;
        }


        public Position MakeStep()
        {
            if (vectors.Count == 0)
                return null;
            
            
            var vector = this.vectors[0];
            vectors.RemoveAt(0);
            Position newPos = StartPosition.Add(vector);

            return newPos;
        }

        public Path DevideWithMinSteps()
        {
            const int maxSteps = 100;
            const int energy = 100;
            Path path = (Path)this.Clone();

            for(int step = 1; step < maxSteps && path.L > energy ; step++)
            {
                path = DevideTo(path, step, energy);
                if (path == null)
                    path = (Path)this.Clone();
            }

            return path;
        }

        public static Path DevideTo(Path lpath, int steps, int energy)
        {

            
            uint maxReduceStep = (uint)Math.Abs(Math.Max(lpath.vectors.Last().X, lpath.vectors.Last().Y));////////module
            
            
            //for(int step = steps; step > 2; step--)
            if(steps == 1)
                return lpath;
            if (steps > 2)
            {
                lpath = DevideTo(lpath, --steps, energy);////ifnull
                if (lpath == null)
                    return null;
                energy -= lpath.L;
            }

            lpath = DevideToTwo(lpath, (uint)energy);
            

            if (lpath.L > energy)
                return null;
            return lpath;
        }

        public static Path DevideToTwo(Path lpath, uint energy)
        {
            uint maxReduceStep = (uint)(Math.Max(Math.Abs(lpath.vectors.Last().X), Math.Abs(lpath.vectors.Last().Y)));////////module
            uint reduceStep = 1;
            Path npath;
            do
            {
                npath = (Path)lpath.Clone();
                var lastVec = npath.vectors.Last();
                var newVec = lastVec.Reduce(reduceStep);
                npath.vectors.Remove(lastVec);
                npath.Add(
                    new Path(newVec
                    , lastVec.Sub(newVec)));
                reduceStep++;
            } while (npath.L > energy && reduceStep <= maxReduceStep / 2);
            return npath;
        }

        public void Add(Path p)
        {
            p.vectors.ForEach(v => this.vectors.Add(v));
        }

        public Position endPos()
        {
            return new Position(vectors.Sum(v => v.X), vectors.Sum(v => v.Y));
        }

        
        public int L
        {
            get
            {
                return vectors.Sum(v => v.L);
            }
        }
    }





    public static class Ext
    {
        public static Position Add(this Position pos0, Position pos1)
        {
            return  new Position(pos0.X + pos1.X, pos0.Y + pos1.Y);
        }

        public static Position Add(this Position pos0, Vector pos1)
        {
            return new Position(pos0.X + pos1.X, pos0.Y + pos1.Y);
        }

        public static Position Sub(this Position pos0, Position pos1)
        {
            return new Position(pos0.X - pos1.X, pos0.Y - pos1.Y);
        }

        public static Position Div(this Position pos0,  int N)
        {
            return new Position(pos0.X / N, pos0.Y/N);
        }



        public static Vector Add(this Vector pos0, Vector pos1)
        {
            return new Vector(pos0.X + pos1.X, pos0.Y + pos1.Y);
        }

        public static Vector Sub(this Vector pos0, Vector pos1)
        {
            return new Vector(pos0.X - pos1.X, pos0.Y - pos1.Y);
        }

        public static Vector Div(this Vector pos0, int N)
        {
            return new Vector(pos0.X / N, pos0.Y / N);
        }

    }
}

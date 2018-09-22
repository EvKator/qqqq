using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using Robot.Common;
using BagirovIslyam.RobotChallange;
using System.Collections.Generic;

namespace BagirovIslyam.RobotChallange.Test
{
    [TestClass]
    public class TestDistanceHelper
    {
        [TestMethod]
        public void FindDentanseTest()
        {
            var p1 = new Position(1, 1);
            var p2 = new Position(2, 4);

            Assert.AreEqual(10, DistanceHelper.FindDistance(p1, p2));

        }

        [TestMethod]
        public void MoveToStationTest()
        {
            Map map = new Map();
            Position p1 = new Position(1,1);
            Position p2 = new Position(2, 2);
            map.Stations.Add(new EnergyStation() { Energy = 2000, Position = p1, RecoveryRate = 2 });

            List<Robot.Common.Robot> robots = new List<Robot.Common.Robot>()
            {
                new Robot.Common.Robot(){Energy=500, Position = p2 }
            };

            IRobotAlgorithm algorithm = new BagirovIslyamAlgorithm();
            RobotCommand step = algorithm.DoStep(robots, 0, map);

            Assert.IsTrue(step is MoveCommand);
            Assert.AreEqual((step as MoveCommand).NewPosition, p1);
        }


        [TestMethod]
        public void CollectEnergyTest()
        {
            Map map = new Map();
            Position p1 = new Position(1, 1);
            Position p2 = new Position(1, 1);
            map.Stations.Add(new EnergyStation() { Energy = 2000, Position = p1, RecoveryRate = 2 });

            List<Robot.Common.Robot> robots = new List<Robot.Common.Robot>()
            {
                new Robot.Common.Robot(){Energy=500, Position = p2 }
            };

            IRobotAlgorithm algorithm = new BagirovIslyamAlgorithm();
            RobotCommand step = algorithm.DoStep(robots, 0, map);

            Assert.IsTrue(step is CollectEnergyCommand);
        }

        [TestMethod]
        public void IsStationFreeTest()
        {
            Map map = new Map();
            Position p1 = new Position(1, 1);
            Position p2 = new Position(1, 1);
            var station = new EnergyStation() { Energy = 2000, Position = p1, RecoveryRate = 2 };
            map.Stations.Add(station);

            List<Robot.Common.Robot> robots = new List<Robot.Common.Robot>()
            {
                new Robot.Common.Robot(){Energy=500, Position = p2 }
            };

            IRobotAlgorithm algorithm = new BagirovIslyamAlgorithm();
            RobotCommand step = algorithm.DoStep(robots, 0, map);

            var sFree = (algorithm as BagirovIslyamAlgorithm).IsStationFree(station, robots[0], robots);

            Assert.IsTrue(sFree is false);
        }


        [TestMethod]
        public void IsCellFreeTest()
        {
            Map map = new Map();
            Position p1 = new Position(1, 1);
            Position p2 = new Position(1, 1);
            var station = new EnergyStation() { Energy = 2000, Position = p1, RecoveryRate = 2 };
            map.Stations.Add(station);

            List<Robot.Common.Robot> robots = new List<Robot.Common.Robot>()
            {
                new Robot.Common.Robot(){Energy=500, Position = p2 }
            };

            IRobotAlgorithm algorithm = new BagirovIslyamAlgorithm();
            RobotCommand step = algorithm.DoStep(robots, 0, map);

            var sFree = (algorithm as BagirovIslyamAlgorithm).IsStationFree(station, robots[0], robots);

            Assert.IsFalse(sFree);
        }


        [TestMethod]
        public void FindNearsestStationTest()
        {
            Map map = new Map();
            Position ps1 = new Position(3, 6);
            Position ps2 = new Position(3, 1);
            Position pr = new Position(3,4);
            var station1 = new EnergyStation() { Energy = 2000, Position = ps1, RecoveryRate = 2 };
            var station2 = new EnergyStation() { Energy = 2000, Position = ps2, RecoveryRate = 2 };
            map.Stations.Add(station1);
            map.Stations.Add(station2);
            List<Robot.Common.Robot> robots = new List<Robot.Common.Robot>()
            {
                new Robot.Common.Robot(){Energy=500, Position = pr }
            };

            IRobotAlgorithm algorithm = new BagirovIslyamAlgorithm();

            RobotCommand step = algorithm.DoStep(robots, 0, map);

            var nearest = (algorithm as BagirovIslyamAlgorithm).FindNearestFreeStation(robots[0], map, robots);

            Assert.AreEqual(nearest, ps1);
        }

    }
}

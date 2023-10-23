using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using Small_TSP.DataProcessors.Interfaces;
using Small_TSP.DataProcessors;
using Small_TSP.DataModel;
using Small_TSP.Solver;
using Newtonsoft.Json;


namespace TravellingSalesmanProblem;

class Program
{
    public static void Main(string[] args)
    {
        string fileName = "/home/mikhail/RiderProjects/SmallTSP/data/111.json";
        IFileManager fileManager = new FileManager();
        string data = fileManager.Read(fileName);
        ISerializer serializer = new JsonDataManager();

        List<ArcImprovedRoute> distanceMatrix = JsonConvert.DeserializeObject<List<ArcImprovedRoute>>(data);

        SolverORTools solver = new SolverORTools();
        
        int [,] dist = solver.BuildDistanceMatrix(distanceMatrix);

        for (int i = 0; i < 16; i++)
        {
            for (int j = 0; j < 16; j++)
            {
                Console.Write($"{dist[i,j]}\t");
            }
            Console.Write("\n");
        }

        
        (int[,] solution, long objective) = solver.GetSolution(distanceMatrix, 0, 1);

        Console.WriteLine($"{objective}");

    }
}



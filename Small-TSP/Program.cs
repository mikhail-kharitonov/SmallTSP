﻿using Small_TSP.DataProcessors.Interfaces;
using Small_TSP.DataProcessors;
using Small_TSP.DataModel;
using Small_TSP.Solver;



namespace TravellingSalesmanProblem;

class Program
{
    public static void Main(string[] args)
    {
        string fileName = "C:\\Users\\mikhail\\Axelot\\Small-TSP\\data\\111.json";//"/home/mikhail/RiderProjects/SmallTSP/data/111.json";
        IFileManager fileManager = new FileManager();
        string data = fileManager.Read(fileName);
        ISerializer serializer = new JsonDataManager();

        List<ArcImprovedRoute> arcsImprovedRoutes = serializer.Deserialize<List<ArcImprovedRoute>>(data);

        SolverORTools solver = new SolverORTools();
        string maskStart = "55,809762_37,392311";//"55,730715_37,395483";//
        string maskEnd = "55,764592_37,877805";
        
        List<int> solution = solver.GetMaskRoutePoints(arcsImprovedRoutes, maskStart, maskEnd);
        Console.WriteLine($"\nMy solution");
        foreach (int item in solution)
        {
            Console.Write($"{item} -> ");
        }

    }
}



namespace Small_TSP.DataProcessors.Interfaces;

public interface ISerializer
{
    T Deserialize<T>(string data);

    string Serialize<T>(T obj);
}
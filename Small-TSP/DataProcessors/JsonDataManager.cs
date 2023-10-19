using Small_TSP.DataProcessors.Interfaces;
using Newtonsoft.Json;
namespace Small_TSP.DataProcessors;

public class JsonDataManager : ISerializer
{
    public T Deserialize<T>(string data)
    {
        return JsonConvert.DeserializeObject<T>(data)!;
    }

    public string Serialize<T>(T obj)
    {
        return JsonConvert.SerializeObject(obj);
    }
}
using UnityEngine;

// Scale the visual only: world-space dynamics, circles and trails stay in metres.
[RequireComponent(typeof(BoatDynamics2D))]
public sealed class BoatHullVisual : MonoBehaviour
{
    private BoatDynamics2D boat;
    private SpriteRenderer visual;
    private LineRenderer hull;
    private Material material;
    void Start()
    {
        boat = GetComponent<BoatDynamics2D>();
        var child = transform.Find("BoatVisual");
        var rootSprite = GetComponent<SpriteRenderer>();
        if (child == null && rootSprite != null)
        {
            var visualObject = new GameObject("BoatVisual"); visualObject.transform.SetParent(transform, false);
            child = visualObject.transform;
            var copy = visualObject.AddComponent<SpriteRenderer>();
            copy.sprite = rootSprite.sprite; copy.sharedMaterial = rootSprite.sharedMaterial;
            copy.color = rootSprite.color; copy.sortingLayerID = rootSprite.sortingLayerID; copy.sortingOrder = rootSprite.sortingOrder;
        }
        visual = child == null ? null : child.GetComponent<SpriteRenderer>();
        if (child != null && rootSprite != null) rootSprite.enabled = false;
        material = new Material(Shader.Find("Sprites/Default"));
        var go = new GameObject("BoatCircularHull"); go.transform.SetParent(transform, false);
        hull = go.AddComponent<LineRenderer>(); hull.sharedMaterial = material;
        hull.useWorldSpace = true; hull.positionCount = 64; hull.loop = true; hull.sortingOrder = 5;
        hull.startColor = hull.endColor = new Color(0.2f, 0.9f, 1, 0.7f);
        hull.startWidth = hull.endWidth = 0.25f;
        LateUpdate();
    }
    void LateUpdate()
    {
        if (boat == null) return;
        if (visual != null && visual.sprite != null)
        {
            var size = visual.sprite.bounds.size;
            visual.transform.localScale = new Vector3(boat.lengthMetres / size.x, boat.beamMetres / size.y, 1);
        }
        if (hull == null) return;
        for (int i = 0; i < hull.positionCount; i++)
        {
            float angle = i * 2 * Mathf.PI / hull.positionCount;
            hull.SetPosition(i, new Vector3(boat.Position.x + Mathf.Cos(angle) * boat.CollisionRadius,
                boat.Position.y + Mathf.Sin(angle) * boat.CollisionRadius, -0.03f));
        }
    }
    void OnDestroy() { if (material != null) Destroy(material); }
}

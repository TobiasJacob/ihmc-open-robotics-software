package us.ihmc.gdx.simulation.scs2;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.gizmo.DynamicGDXModel;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;

import java.util.ArrayList;

public class GDXSimulatedTerrainObject
{
   private final TerrainObjectDefinition terrainObjectDefinition;
   private final ArrayList<ModelInstance> modelInstances = new ArrayList<>();

   public GDXSimulatedTerrainObject(TerrainObjectDefinition terrainObjectDefinition)
   {
      this.terrainObjectDefinition = terrainObjectDefinition;
   }

   public void create()
   {
      for (DynamicGDXModel terrainModelPart : GDXVisualTools.collectNodes(terrainObjectDefinition.getVisualDefinitions()))
      {
         ModelInstance modelInstance = terrainModelPart.getOrCreateModelInstance();
         modelInstances.add(modelInstance);
         GDXTools.toGDX(terrainModelPart.getLocalTransform(), modelInstance.transform);
      }
   }

   public void getRealRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (ModelInstance modelInstance : modelInstances)
      {
         modelInstance.getRenderables(renderables, pool);
      }
   }
}

namespace MySceneGraph
open Aardvark.Rendering
open MySceneGraph
open FSharp.Data.Adaptive
open Aardvark.Rendering.GL
open Aardvark.Base
open System
open MySceneGraph

[<AutoOpen>]
module Bklasndkoa =

        let CreateLodRendererNew(x : Runtime, config : LodRendererConfig, data : aset<LodTreeInstance>) =

            let preparedState = PreparedPipelineState.ofPipelineState config.fbo x.ResourceManager config.surface config.state

            //let info : LodRenderingInfo =
            //    {
            //        LodRenderingInfo.quality = quality
            //        LodRenderingInfo.maxQuality = maxQuality
            //        LodRenderingInfo.renderBounds = renderBounds
            //    }

            new LodRenderer(x.Context, x.ResourceManager, preparedState, config, data) :> IPreparedRenderObject
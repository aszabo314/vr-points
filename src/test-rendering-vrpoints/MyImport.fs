namespace MySceneGraph


open System
open System.Collections.Generic
open System.Threading
open System.Threading.Tasks
open Aardvark.Geometry
open Aardvark.Geometry.Points
open Aardvark.Data.Points
open Aardvark.Data.Points.Import
open Aardvark.Base
open Aardvark.Rendering
open FSharp.Data.Adaptive
open Aardvark.Rendering.PointSet.LodTreeInstance

[<AutoOpen>]
module Importy = 
    let myImport store cache key = 
        let store = PointCloud.OpenStore(store, cache)
        
        let key = 
            if String.IsNullOrEmpty key then
                try
                    let test = store.GetByteArray("Index")
                    if isNull test then failwith "no key given"
                    else System.Text.Encoding.Unicode.GetString(test)
                with _ ->
                    failwith "no key given"
            else
                key
        
        let root =
            try
                let set = store.GetPointSet(key)

                let points = set

                points.Root.Value
            with e ->
                //Log.error "%A" e
                store.GetPointCloudNode key
        let bounds = root.Cell.BoundingBox

        let trafo = Similarity3d(1.0, Euclidean3d(Rot3d.Identity, -bounds.Center))
        let source = Symbol.Empty
        let root = PointTreeNode.Create(System.Guid.NewGuid(), root, store.Cache, source, trafo, None, None, 0, root) 
        match root with
        | Some root ->
            let uniforms = MapExt.ofList []
            let uniforms = MapExt.add "Scales" (AVal.constant V4d.IIII :> IAdaptiveValue) uniforms
            Some { 
                root = root
                uniforms = uniforms
            }
        | None ->
            None
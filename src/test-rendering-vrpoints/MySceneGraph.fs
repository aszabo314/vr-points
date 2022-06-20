namespace MySceneGraph

open Aardvark.Base
open FSharp.Data.Adaptive
open Aardvark.SceneGraph
open Aardvark.Rendering
open MySceneGraph

[<AutoOpen>]
module LodTreeRendering =

    type LodTreeRenderConfig =
        {
            budget : aval<int64>
            splitfactor : aval<float>
            time : aval<System.DateTime>
            maxSplits : aval<int>
            renderBounds : aval<bool>
            stats : cval<LodRendererStats>
            pickTrees : Option<cmap<ILodTreeNode,SimplePickTree>>
            alphaToCoverage : bool
            views : aval<Trafo3d[]>
            projs : aval<Trafo3d[]>
        }

    module LodTreeRenderConfig =
        let private time =
            let sw = System.Diagnostics.Stopwatch.StartNew()
            let start = System.DateTime.Now

            let self = ref Unchecked.defaultof<aval<System.DateTime>>

            self :=
                AVal.custom (fun t -> 
                    let now = start + sw.Elapsed
                    AdaptiveObject.RunAfterEvaluate (fun () -> self.Value.MarkOutdated())
                    now
                )
            !self

        let simple =
            {
                budget = AVal.constant -1L
                splitfactor = AVal.constant 0.4
                time = time
                maxSplits = AVal.constant System.Environment.ProcessorCount
                renderBounds = AVal.constant false
                stats = AVal.init Unchecked.defaultof<_>
                pickTrees = None
                alphaToCoverage = false
                views = AVal.constant [||]
                projs = AVal.constant [||]
            }

    module Sg = 
        type LodTreeNode(stats : cval<LodRendererStats>, views : aval<Trafo3d[]>, projs : aval<Trafo3d[]>, pickTrees : Option<cmap<ILodTreeNode,SimplePickTree>>, alphaToCoverage : bool, budget : aval<int64>, splitfactor : aval<float>, renderBounds : aval<bool>, maxSplits : aval<int>, time : aval<System.DateTime>, clouds : aset<LodTreeInstance>) =
            member x.Time = time
            member x.Clouds = clouds
            member x.MaxSplits = maxSplits
            
            member x.Views = views
            member x.Projs = projs
            member x.Stats = stats
            member x.PickTrees = pickTrees
            member x.RenderBounds = renderBounds
            member x.Budget = budget
            member x.AlphaToCoverage = alphaToCoverage
            member x.SplitFactor = splitfactor
            interface ISg

        let lodTree (cfg : LodTreeRenderConfig) (data : aset<LodTreeInstance>) =
            LodTreeNode(cfg.stats, cfg.views, cfg.projs, cfg.pickTrees, cfg.alphaToCoverage, cfg.budget, cfg.splitfactor, cfg.renderBounds, cfg.maxSplits, cfg.time, data) :> ISg
    

namespace Aardvark.SceneGraph.Semantics

open Aardvark.Base
open Aardvark.Rendering
open FSharp.Data.Adaptive
open Aardvark.SceneGraph  
open MySceneGraph

[<Rule>]
type LodNodeSem() =
    member x.RenderObjects(sg : Sg.LodTreeNode, scope : Ag.Scope) =
        let state = PipelineState.ofScope scope
        let surface = scope.Surface
        let pass = scope.RenderPass

        let model = scope.ModelTrafo
        let view = scope.ViewTrafo
        let proj = scope.ProjTrafo

        let id = newId()
        let obj =
            { new ILodRenderObject with
                member x.Id = id
                member x.AttributeScope = scope
                member x.RenderPass = pass
                member x.Prepare(r, fbo) = 
                    let config =
                        {
                            fbo = fbo
                            time = sg.Time
                            surface = surface
                            state = state
                            pass = pass
                            model = model
                            views = sg.Views
                            projs = sg.Projs
                            budget = sg.Budget
                            renderBounds = sg.RenderBounds
                            maxSplits = sg.MaxSplits
                            splitfactor = sg.SplitFactor
                            stats = sg.Stats
                            pickTrees = sg.PickTrees
                            alphaToCoverage = sg.AlphaToCoverage
                        }

                    MySceneGraph.Bklasndkoa.CreateLodRendererNew(r :?> Aardvark.Rendering.GL.Runtime, config, sg.Clouds)
            }

        ASet.single (obj :> IRenderObject)
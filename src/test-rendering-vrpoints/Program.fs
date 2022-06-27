open System
open System.IO
open Aardvark.Base
open Aardvark.Rendering
open Aardvark.SceneGraph
open Aardvark.Application
open Aardvark.Application.Slim
open FSharp.Data.Adaptive
open FSharp.Data.Adaptive.Operators
open Aardvark.Application.Utilities
open Aardvark.Rendering.PointSet
open Aardvark.Base.Ag
open Aardvark.Geometry.Points
open Aardvark.Data.Points
open Aardvark.Data.Points.Import
open MySceneGraph
open Valve.VR

open VRVis.Vgm
open PointCloudSegmentation
open PointCloudSegmentation.DynamicSegmentation
open System.Threading

//let pois =
//    [
//        //"2020078-Eltville_store",1,V3d(3437123.4634,5543482.1291,91.4592),"Buero"
//    ]
//    |> List.map (fun (pc,num,loc,name) -> (pc,num),(loc,name))
//    |> Map.ofList

let clouds =
    [
        @"D:\jb_large_store", None, "jb_large"
    ]
    |> List.map (fun (path,key : Option<string>,name) -> name,(path,key))
    |> Map.ofList

let pois =
    [
        "jb_large",Map.ofList [1,V3d(0.0000,0.0000,0.0000); 2,V3d(14.5369,-11.3891,2.2123)]
    ]
    |> Map.ofList
let cache = new LruDictionary<_,_>(1<<<20)

let get' (store : string) (key : Option<string>) =
    let key = 
        match key with 
        | Some k -> k
        | None -> 
            let k = Path.combine [(if System.IO.Directory.Exists store then store else System.IO.Path.GetDirectoryName store);"key.txt"] |> File.readAllText
            k.Trim([|' ';'\t';'\r';'\n'|])
    Importy.myImport store cache key |> Option.get
let get (store : string) =
    get' store None

[<EntryPoint;STAThread>]
let main argv = 
    Aardvark.Init()

    use app = new Aardvark.Application.OpenVR.OpenGlVRApplicationLayered(4, false)
    
    let curKey = cval ((clouds |> Seq.head).Key)
    let curStore = curKey |> AVal.map (fun k -> Map.find k clouds)

    let inst = 
        curStore |> AVal.map (fun (store,key) -> 
            [
                match key with 
                | Some k -> yield get' store (Some k)
                | None -> yield get store
            ]
         ) |> AList.ofAVal
    let roots = inst |> AList.map (fun t -> (t.root.Id :?> IPointCloudNode))
    let centroid = roots |> AList.mapi (fun i root -> V3d root.CentroidLocal + root.Cell.GetCenter())


    let color = cval true
    let pointSize = cval 5.0
    let planeFit = cval true
    let planeFitRadius = cval 30.0

    

    let velocity = 
        app.System.ConnectedControllers |> ASet.toAVal |> AVal.map (fun c ->
            c |> HashSet.toArray
            |> Array.map (fun c ->
                let trigger = c.Axis.[1]
                (c.MotionState.Pose,trigger.Position) ||> AVal.map2 (fun t v ->
                    match v with
                    | Some v ->
                        t.Forward.TransformDir -V3d.OIO * v.X * 5.0
                    | None ->
                        V3d.Zero
                )
            )
        )

    let velocity =
        AVal.custom (fun t ->
            let arr = velocity.GetValue t
            arr |> Array.sumBy (fun v -> v.GetValue t)
        )

    let sw = System.Diagnostics.Stopwatch.StartNew()
    let t0 = DateTime.Now
    let t = app.Time |> AVal.map (fun _ -> t0 + sw.Elapsed)

    let customT : ref<Option<V3d>> = ref None
    let f = 
        let oldState = ref Unchecked.defaultof<_> 
        let res =
            AdaptiveFunc<V3d>(fun res o ->
                let s = t.GetValue res
                let dt : TimeSpan = s - !oldState
                let newA = 
                    match !customT with 
                    | None -> 
                        let v = velocity.GetValue res
                        o + v * dt.TotalSeconds
                    | Some custom -> 
                        customT.Value <- None 
                        custom
                oldState := s 
                newA
            )
        oldState := t.GetValue()
        lock t (fun () ->
            t.Outputs.Add res |> ignore
        )

        AVal.constant res

    
    let t = AVal.integrate V3d.Zero app.Time [f]
    
    let s = cval 1.0
    
    let pointCloudtrafo = 
        adaptive {
            let! t = t
            let! s = s
            let! cs = (AList.toAVal centroid)
            return 
                Trafo3d.Translation -(List.average (cs |> IndexList.toList)) *  
                Trafo3d.Translation t *
                Trafo3d.Scale (V3d.III * s) *
                Trafo3d.Identity
        }
        
    let segmentationResult = cval None
    let cancelAndResetSegmentation (cts : Option<CancellationTokenSource>) =
        transact (fun _ -> segmentationResult.Value <- None)
        cts |> Option.iter (fun c -> c.Cancel())
    
    let doDynamicSegmentation (cts : CancellationTokenSource) (seed : V3d) =
        try 
            let cfg : PointCloudSegmentation.Config =
                {
                    radius = 0.5
                    planeTolerance = 0.15
                    normalTolerance = 12.0
                    maxCount = 15
                    distanceLimit = 20.0
                    desiredPpm = 600.0
                }
            let cb (state : SegmentationState) (res : SegmentationResult) =
                if not cts.IsCancellationRequested then 
                    Log.line "DynSeg result %d" (res.Planes |> Array.sumBy (fun pi -> pi.pointCount))
                    transact (fun _ -> segmentationResult.Value <- Some res)
                else 
                    Log.line "cancelled"
            let seed : SegmentationSeed = SegmentationSeed.Sphere(Sphere3d(seed, 0.45))
            let nodes = roots |> AList.toAVal |> AVal.force |> IndexList.toList |> List.map (fun r -> Trafo3d.Identity, r)
            DynamicSegmentation.segmentation' [||] cfg cts.Token None cb seed nodes |> Log.line "DynSeg return %d"
        with e -> 
            Log.error "DynSeg e: %A" e

    let segmentationMvar : MVar<Option<V3d>> = MVar.empty()
    let pickPointAndDoSegmentation (controllerPos : Option<V3d>) =
        let p = controllerPos |> Option.map (fun cp -> pointCloudtrafo.GetValue().Backward.TransformPos cp)
        segmentationMvar.Put p
        Log.line "DynSeg enqueue at %A" p

    let segmentationPuller = 
        startThread <| fun () -> 
            let rec awaitNextSignal (lastCts : Option<CancellationTokenSource>) =
                match segmentationMvar.Take() with
                | Some segLoc -> 
                    cancelAndResetSegmentation lastCts
                    use cts = new CancellationTokenSource()
                    Log.line "DynSeg run at %A" segLoc
                    (startThread <| fun () -> doDynamicSegmentation cts segLoc) |> ignore
                    awaitNextSignal (Some cts)
                | None -> 
                    cancelAndResetSegmentation lastCts
                    Log.line "DynSeg cleared"
                    awaitNextSignal None
            awaitNextSignal None
                

    let centroid = ()
    use sub = 
        app.System.ConnectedControllers.AddCallback(fun _ d ->
            for d in d do
                match d with
                | Add(_, c) ->
                    let touchpad = c.Axis.[0]
                    let trigger = c.Axis.[1]
                    

                    let mutable last = None
                    touchpad.Press.Add (fun () ->
                        transact (fun () -> color.Value <- not color.Value)
                    )
                    touchpad.Touch.Add (fun () ->
                        last <- AVal.force touchpad.Position
                    )
                    touchpad.UnTouch.Add (fun () ->
                        last <- None
                    )
                    touchpad.Position.AddCallback(fun p ->
                        match p with
                        | Some p when p <> V2d.Zero ->
                            match last with
                            | Some l ->
                                let delta : V2d = 10.0 * (p - l)
                                transact (fun () -> 
                                    pointSize.Value <- clamp 0.1 15.0 (pointSize.Value + delta.Y)
                                    let newpfr = clamp 1.0 17.5 (planeFitRadius.Value + delta.X)
                                    planeFitRadius.Value <- newpfr
                                )
                                
                            | None ->
                                ()
                            last <- Some p
                        | _ ->
                            last <- None
                    ) |> ignore

                    c.Events.Add(fun evt ->
                        let eventType = evt.eventType |> int |> unbox<EVREventType>
                        match eventType with
                        | EVREventType.VREvent_ButtonPress -> 
                            match evt.data.controller.button with
                            | 1u -> 
                                let loc = c.MotionState.Pose.GetValue().Forward.C3.XYZ
                                //printfn "burger %A" loc
                                pickPointAndDoSegmentation (Some loc)
                            | 2u ->  //squeeze
                                //transact (fun () -> planeFit.Value <- not planeFit.Value)
                                ()
                            | _ -> ()
                        | EVREventType.VREvent_ButtonUnpress -> 
                            match evt.data.controller.button with
                            | 1u -> 
                                //burger unpress
                                pickPointAndDoSegmentation None
                            | _ -> ()
                        | _ ->
                            ()
                    )
                    ()
                | _ ->
                    ()
        )
    let controllerSgs = 
        Sg.set (app.System.ConnectedControllers |> ASet.choose (fun c -> 
            c.Model |> Option.map (fun m -> 
                m |> Sg.trafo c.MotionState.Pose
            )
            
        ))
        |> Sg.shader {
            do! DefaultSurfaces.trafo
            do! DefaultSurfaces.diffuseTexture
        }
        |> Sg.uniform "ViewTrafo" app.Info.viewTrafos
        |> Sg.uniform "ProjTrafo" app.Info.projTrafos


    let cfg = { LodTreeRenderConfig.simple with views = app.Info.viewTrafos; projs = app.Info.projTrafos }

    let getPlayerT() = 
        t.GetValue()
    let setPlayerT i =
        try
            let name = curKey.GetValue()
            let t = pois.[name].[i]
            customT.Value <- Some t
            Log.line "set player pos %s to %d %A" name i t
        with e -> 
            Log.error "poi not found: %d" i
            ()

    let reset() =
        Log.line "reset"
        transact (fun _ -> 
            s.Value <- 1.0
        )


    let segmentationSg = 
        (SegmentationSceneGraph.scene 
            segmentationResult 
            (pointCloudtrafo |> AVal.map (fun t -> t.Inverse)) 
            (app.Info.viewTrafos |> AVal.map (Array.item 0))
            (AVal.constant app.DesiredSize))
        |> Sg.uniform "ViewTrafo" app.Info.viewTrafos
        |> Sg.uniform "ProjTrafo" app.Info.projTrafos



    let vis =
        color |> AVal.map (function
            | true -> PointVisualization.Color
            | false -> PointVisualization.None
        )


    let pointCloudSg = 
        Sg.lodTree cfg (ASet.ofAList inst)
        |> Sg.shader {
            do! PointSetShaders.lodPointSize2
            do! PointSetShaders.lodPointCircular
        }
        |> Sg.trafo pointCloudtrafo
        |> Sg.uniform "PointSize" pointSize
        |> Sg.uniform' "ViewportSize" app.DesiredSize
        |> Sg.uniform "PointVisualization" vis
        |> Sg.uniform "MagicExp" (~~0.0)
        |> Sg.uniform "ViewTrafo" app.Info.viewTrafos
        |> Sg.uniform "ProjTrafo" app.Info.projTrafos


    let fboSig =
        let s = app.FramebufferSignature
        app.Runtime.CreateFramebufferSignature([DefaultSemantic.Colors, TextureFormat.Rgba8; DefaultSemantic.DepthStencil, TextureFormat.Depth24Stencil8], 1, 2, s.PerLayerUniforms)
        
    let render (sg : ISg) =

        let clear = app.Runtime.CompileClear(fboSig, clear { color C4f.Black; depth 1.0; stencil 0 })
        let task = app.Runtime.CompileRender(fboSig, sg)

        let color = app.Runtime.CreateTexture2DArray(app.DesiredSize, TextureFormat.Rgba8, 1, 1, 2)
        let depth = app.Runtime.CreateTexture2DArray(app.DesiredSize, TextureFormat.Depth24Stencil8, 1, 1, 2)
        let fbo = 
            app.Runtime.CreateFramebuffer(
                fboSig, 
                [
                    DefaultSemantic.Colors, color.[TextureAspect.Color, 0, *] :> IFramebufferOutput
                    DefaultSemantic.DepthStencil, depth.[TextureAspect.DepthStencil, 0, *] :> IFramebufferOutput
                ]
            )

        AVal.custom (fun token ->
            let o = OutputDescription.ofFramebuffer fbo
            clear.Run(token, RenderToken.Empty, o)
            task.Run(token, RenderToken.Empty, o)
            color, depth
        )


    let pass0 = render pointCloudSg
    
    let randomTex = 
        let img = PixImage<float32>(Col.Format.RGB, V2i.II * 512)

        let rand = RandomSystem()
        img.GetMatrix<C3f>().SetByCoord (fun _ ->
            V3d(rand.UniformV2dDirection(), 0.0).ToC3d().ToC3f()
        ) |> ignore

        app.Runtime.PrepareTexture(PixTexture2d(PixImageMipMap [| img :> PixImage |], TextureParams.empty)) :> ITexture
            
    let nearFar =
        app.Info.projTrafos |> AVal.map (fun t ->
            let p = t.[0].Backward
            let n = p.TransformPosProj(V3d.OON).Z |> abs
            let f = p.TransformPosProj(V3d.OOI).Z |> abs

            V2d(n, f)
        )

        
    let planeFitTol = AVal.constant 0.05
    let deferredSg =
        Sg.fullScreenQuad
        |> Sg.shader {
            do! PointSetShaders.blitPlaneFit
        }
        |> Sg.texture "Colors" (AVal.map fst pass0)
        |> Sg.texture "DepthStencil" (AVal.map snd pass0)
        |> Sg.uniform "ViewTrafo" app.Info.viewTrafos
        |> Sg.uniform "ProjTrafo" app.Info.projTrafos
        
        |> Sg.uniform "NearFar" nearFar
        |> Sg.uniform "PlaneFit" planeFit
        |> Sg.uniform "RandomTexture" (AVal.constant randomTex)
        |> Sg.uniform "Diffuse" (AVal.constant true)
        |> Sg.uniform' "ViewportSize" app.DesiredSize
        |> Sg.uniform "PlaneFitTolerance" planeFitTol
        |> Sg.uniform "PlaneFitRadius" planeFitRadius

    let sg = 
        Sg.ofList
            [
                deferredSg
                segmentationSg
                controllerSgs
            ]



    let thread =
        startThread <| fun () ->
            while true do
                let l = Console.ReadKey()
                let v = app.Info.viewTrafos |> AVal.force
                let f = v.[0].Backward.C2.XYZ * V3d.IIO |> Vec.normalize
                let r = V3d(f.Y, -f.X, 0.0)
                let u = V3d.OOI
                match l.KeyChar with
                //| 'd' -> transact (fun () -> t.Value <- t.Value + 0.5 * r)
                //| 'a' -> transact (fun () -> t.Value <- t.Value - 0.5 * r)
                //| 'w' -> transact (fun () -> t.Value <- t.Value + 0.5 * f)
                //| 's' -> transact (fun () -> t.Value <- t.Value - 0.5 * f)
                //| 'c' -> transact (fun () -> t.Value <- t.Value + 0.25 * u)
                //| 'e' -> transact (fun () -> t.Value <- t.Value - 0.25 * u)
                | 't' -> transact (fun () -> s.Value <- s.Value + 0.02)
                | 'r' -> transact (fun () -> s.Value <- s.Value - 0.02)
                | 'm' -> printfn "\"%s\",%s" (curKey.GetValue()) (getPlayerT() |> (fun v -> sprintf "V3d(%.4f,%.4f,%.4f)" v.X v.Y v.Z))
                | 'l' -> reset()
                | '1' -> setPlayerT 1
                | '2' -> setPlayerT 2
                //| '3' -> setPlayerPosToPoi 3
                //| '4' -> setPlayerPosToPoi 4
                //| '5' -> setPlayerPosToPoi 5
                //| '6' -> setPlayerPosToPoi 6
                //| '7' -> setPlayerPosToPoi 7
                //| '8' -> setPlayerPosToPoi 8
                //| '9' -> setPlayerPosToPoi 9
                //| '0' -> setPlayerPosToPoi 0
                | _ -> ()
    
    app.RenderTask <- app.Runtime.CompileRender(app.FramebufferSignature, sg)
    app.Run()
    0

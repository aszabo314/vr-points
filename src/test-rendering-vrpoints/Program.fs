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
        @"D:\3278_5511_0_10\pointcloud\data.bin", (Some "3278_5511_0_10"), "koeln"
        @"D:\erzberg_toni\data.bin", None, "erzberg_toni"
        @"D:\store_BLK ARC-2050303-42\BLK ARC-2050303-42", None, "georg_wald"
        @"D:\store_17-03-2022_scan-41\17-03-2022_scan-41 1", None, "georg_haus"
        @"D:\innen_store\data.bin", None, "techzentrum"
        @"D:\KaunertalStore", None, "kaunertal"
        @"D:\store3\data.bin", None, "himmelreich"
    ]
    |> List.map (fun (path,key : Option<string>,name) -> name,(path,key))
    |> Map.ofList

let pois =
    [
        "jb_large",Map.ofList 
            [
                1,("origin",V3d(0.0000,0.0000,0.0000))
                2,("cpunz",V3d(14.5369,-11.3891,2.2123))
                3,("alien",V3d(-16.7355,-6.8227,-0.3040))
                4,("eingangstuer",V3d(-8.9254,0.5787,0.4679))
                5,("schaukel",V3d(9.8840,5.8154,3.3978))
                6,("dach",V3d(4.2659,5.1292,-6.7622))
            ]
        "erzberg_toni",Map.ofList 
            [
                1,("origin",V3d.Zero)
                2,("haupttunnel",V3d(-26.8944,-48.1361,3.8753))
                3,("fahrzeuge_weiss",V3d(-80.0564,-95.1948,4.6572))
                4,("fahrzeuge_orange",V3d(-82.1689,-62.9014,4.3702))
                5,("eingang_schnee",V3d(16.0356,291.3683,-0.0592))
                6,("creepy_gang",V3d(-118.5345,297.4212,-0.3954))
                7,("vogelperspektive",V3d(58.7516,16.9669,-94.4066))
                8,("servicegang",V3d(106.5624,-30.9442,1.5828))
                9,("creepy_eingang",V3d(-122.9496,372.6927,-1.9341))
            ]
        "georg_wald",Map.ofList 
            [
                1,("origin",V3d.Zero)
                2,("huegel",V3d(-24.8602,12.9335,3.3191))
                3,("auto",V3d(-149.5556,95.8487,51.4895))
                4,("vogelperspektive",V3d(-39.4971,14.2139,-60.3953))
                5,("huegel_oben",V3d(138.7763,-162.5310,-55.6492))
                6,("kurve",V3d(-82.3435,112.0376,41.6321))
            ]
        "georg_haus",Map.ofList 
            [
                1,("origin",V3d.Zero)
                2,("kueche",V3d(3.2566,-0.8960,1.6695))
                3,("wohnbereich",V3d(-1.0947,-0.5023,1.5162))
                4,("kinderzimmer",V3d(-3.9321,-1.1939,1.5372))
                5,("schlafzimmer",V3d(-4.0844,2.2880,1.5132))
            ]
        "koeln",Map.ofList 
            [
                1,("originstrasse",V3d(3.7907,2.3028,0.7818))
                2,("vogelperspektive",V3d(-14.5481,-187.1479,-149.0140))
                3,("unter_bruecke",V3d(79.5184,-470.5748,3.6630))
                4,("kreisverkehr_gehsteigkante",V3d(379.9717,-381.6181,1.9463))
                5,("fussgaengerbruecke",V3d(418.2195,-473.1021,2.3969))
                6,("spielplatz",V3d(-162.6086,-72.1859,4.2214))
                7,("kreuzung",V3d(317.0050,14.7436,0.8331))
                8,("orange_strasse",V3d(231.3207,-290.0860,1.1631))
            ]
        "techzentrum",Map.ofList 
            [
                1,("origin",V3d.Zero)
                2,("rotes_sofa",V3d(-6.5314,-2.8540,1.0798))
                3,("empfang",V3d(-6.1924,4.2434,1.1334))
                4,("cpunz",V3d(-27.3736,1.8913,1.1070))
                5,("strasse",V3d(-26.5605,-9.3756,2.5877))
                6,("auto",V3d(-12.7791,-15.4265,3.9160))
                7,("feuerloescher",V3d(7.9868,2.6521,1.1194))
                8,("spielzimmer",V3d(19.6353,-0.8480,1.0997))
                9,("spielzimmer_menschen",V3d(18.0555,3.3676,1.0902))
            ]
        "kaunertal",Map.ofList 
            [
                1,("origin",V3d.Zero)
                2,("vogelperspektive",V3d(-60.8245,36.0486,-58.7428))
                3,("runtergucken",V3d(21.4004,-22.8059,-48.7169))
                4,("bagger",V3d(58.7386,60.7998,-25.0895))
                5,("steinwand",V3d(8.6249,43.9014,-8.3841))
                6,("steil",V3d(-37.8224,13.9814,34.8854))
                7,("unten",V3d(-104.0170,2.5848,91.4539))
            ]
        "himmelreich",Map.ofList 
            [
                1,("origin",V3d.Zero)
            ]
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
    
    let curKey = cval ("jb_large")
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
    let planeFitRadius = cval 12.5

    let currentLocalPointerRay =
        app.System.ConnectedControllers |> ASet.toAVal |> AVal.bind (fun c ->
            let arr = c |> HashSet.toArray
            if arr.Length = 0 then AVal.constant None
            else 
                let c = arr.[0]
                c.MotionState.Pose |> AVal.map (fun t -> 
                    let pos = t.Forward.TransformPos V3d.Zero
                    let dir = t.Forward.TransformDir V3d.OIO
                    Ray3d(pos,dir.Normalized) |> Some
                )
            
        )

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
    let speed = cval 0.0

    let customT : ref<Option<V3d>> = ref None
    let f = 
        let oldState = ref Unchecked.defaultof<_> 
        let res =
            AdaptiveFunc<V3d>(fun res o ->
                let s = t.GetValue res
                let speed = speed.GetValue res
                let dt : TimeSpan = s - !oldState
                let newA = 
                    match !customT with 
                    | None -> 
                        let v = velocity.GetValue res
                        o + speed * v * dt.TotalSeconds
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
        
    let currentLocalPickRay =
        currentLocalPointerRay |> AVal.map (fun r  -> 
            match r with 
            | None -> None
            | Some r -> 
                let p0 = r.Origin
                let p1 = r.Origin + 2.0 * r.Direction
                Some (Line3d(p0,p1))
        )
    let currentWorldPickRay = 
        (currentLocalPickRay,pointCloudtrafo) ||> AVal.map2 (fun r t -> 
            match r with 
            | None -> None
            | Some r -> 
                let p0 = r.P0 |> t.Backward.TransformPos
                let p1 = r.P1 |> t.Backward.TransformPos
                Some (Line3d(p0,p1))
        )

    let getSeed() : Option<V3d> = 
        currentWorldPickRay.GetValue() |> Option.bind (fun line -> 
            let roots = roots |> AList.toAVal |> AVal.force |> IndexList.toList |> List.map (fun r -> r)
            let picks = 
                roots 
                |> List.tryPick (fun r -> 
                    r.QueryPointsNearLineSegment(line,0.1)
                    |> Seq.tryHead
                    |> Option.bind (fun chunk -> 
                        chunk.Positions |> Seq.tryHead
                    )
                )
            picks
        )   

    let segmentationResult = cval None
    let cancelAndResetSegmentation (cts : Option<CancellationTokenSource>) =
        transact (fun _ -> segmentationResult.Value <- None)
        cts |> Option.iter (fun c -> c.Cancel())
    
    let doDynamicSegmentation (cts : CancellationTokenSource) (seed : V3d) =
        try 
            let cfg : PointCloudSegmentation.Config =
                {
                    radius = 0.5
                    planeTolerance = 0.10
                    normalTolerance = 12.0
                    maxCount = 15
                    distanceLimit = 20.0
                    desiredPpm = 900.0
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
    let pickPointAndDoSegmentation (bla : Option<V3d>) =
        //let p = controllerPos |> Option.map (fun cp -> pointCloudtrafo.GetValue().Backward.TransformPos cp)
        segmentationMvar.Put bla
        Log.line "DynSeg enqueue"

    let segmentationPuller = 
        startThread <| fun () -> 
            let rec awaitNextSignal (lastCts : Option<CancellationTokenSource>) =
                match segmentationMvar.Take(), getSeed() with
                | Some segLoc, Some seed -> 
                    cancelAndResetSegmentation lastCts
                    use cts = new CancellationTokenSource()
                    Log.line "DynSeg run"
                    (startThread <| fun () -> doDynamicSegmentation cts seed) |> ignore
                    awaitNextSignal (Some cts)
                | _ -> 
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
                                    pointSize.Value <- clamp 0.1 10.0 (pointSize.Value + delta.Y)
                                    let newpfr = clamp 1.0 12.5 (planeFitRadius.Value + delta.X)
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
                                //printfn "burger %A" loc
                                pickPointAndDoSegmentation (Some V3d.Zero)
                            | 2u ->  //squeeze
                                //transact (fun () -> planeFit.Value <- not planeFit.Value)
                                pickPointAndDoSegmentation None
                            | _ -> ()
                        | EVREventType.VREvent_ButtonUnpress -> 
                            match evt.data.controller.button with
                            | 1u -> 
                                //burger unpress
                                //pickPointAndDoSegmentation None
                                ()
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
            let (pn,t) = pois.[name].[i]
            customT.Value <- Some t
            Log.line "set player pos %s to %d %s %A" name i pn t
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

    let localPickSg =
        let onoff = 
            currentLocalPickRay |> AVal.map Option.isSome
        Sg.lines (AVal.constant C4b.Red) (currentLocalPickRay |> AVal.map (fun l -> l |> Option.map (fun l -> Array.singleton l) |> Option.defaultValue Array.empty))
        |> Sg.shader {
            do! DefaultSurfaces.trafo
            do! DefaultSurfaces.thickLine
            do! DefaultSurfaces.vertexColor
        }
        //|> Sg.onOff onoff
        |> Sg.uniform "LineWidth" (AVal.constant 10.0)
        |> Sg.uniform "ViewTrafo" app.Info.viewTrafos
        |> Sg.uniform "ProjTrafo" app.Info.projTrafos
        //|> Sg.depthTest (AVal.constant DepthTest.None)


    let sg = 
        Sg.ofList
            [
                deferredSg
                segmentationSg
                controllerSgs
                localPickSg
            ]

    let printInfo() =
        Log.line "PointClouds:"
        let curkey = curKey.GetValue()
        let cloudKeys = 
            [
                "jb_large","P"
                "koeln","O"
                "erzberg_toni","I"
                "georg_wald","U"
                "georg_haus","Z"
                "techzentrum","T"
                "kaunertal","R"
                "himmelreich","E"
            ] |> Map.ofList
        for k in clouds  |> Map.toArray |> Array.map fst do
            let key = cloudKeys |> Map.find k
            if curkey = k then 
                Log.line " ((%s)) %s" key k
            else 
                Log.line " (%s) %s" key k
        Log.line "Points of Interest for %s:" curkey
        for (number,(name,coord)) in pois |> Map.find curkey |> Map.toSeq do
            Log.line " (%d) %s [%.2f,%.2f,%.2f]" number name coord.X coord.Y coord.Z
        Log.line "(W/S) Speed = %.1f" (speed.GetValue())
        Log.line "(C) Color = %b" (color.GetValue())
        Log.line "(Q) pointrendering defaults"
        Log.line "(X) clear segmentation"
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
                | 'w' -> transact (fun () -> speed.Value <- clamp 0.0 100.0 (speed.Value + 0.5)); Log.line "speed=%.1f" speed.Value
                | 's' -> transact (fun () -> speed.Value <- clamp 0.0 100.0 (speed.Value - 0.5)); Log.line "speed=%.1f" speed.Value
                | 'c' -> transact (fun () -> color.Value <- not color.Value); Log.line "Color=%b" color.Value
                | 'q' -> transact (fun () -> color.Value <- true; pointSize.Value <- 5.0; planeFit.Value <- true; planeFitRadius.Value <- 12.5); Log.line "Pointrendering reset to defaults."
                | 'x' -> pickPointAndDoSegmentation None
                //| 'r' -> transact (fun () -> s.Value <- s.Value - 0.02)
                | 'm' -> printfn "\"%s\",%s" (curKey.GetValue()) (getPlayerT() |> (fun v -> sprintf "V3d(%.4f,%.4f,%.4f)" v.X v.Y v.Z))
                | 'l' -> printInfo()
                | '1' -> setPlayerT 1
                | '2' -> setPlayerT 2
                | '3' -> setPlayerT 3
                | '4' -> setPlayerT 4
                | '5' -> setPlayerT 5
                | '6' -> setPlayerT 6
                | '7' -> setPlayerT 7
                | '8' -> setPlayerT 8
                | '9' -> setPlayerT 9
                | '0' -> setPlayerT 0
                | 'p' -> let s = "jb_large"     in transact (fun _ -> curKey.Value <-  s); Log.line "switched to %s" s
                | 'o' -> let s = "koeln"     in transact (fun _ -> curKey.Value <-  s); Log.line "switched to %s" s
                | 'i' -> let s = "erzberg_toni" in transact (fun _ -> curKey.Value <-  s); Log.line "switched to %s" s
                | 'u' -> let s = "georg_wald"   in transact (fun _ -> curKey.Value <-  s); Log.line "switched to %s" s
                | 'z' -> let s = "georg_haus"   in transact (fun _ -> curKey.Value <-  s); Log.line "switched to %s" s
                | 't' -> let s = "techzentrum"   in transact (fun _ -> curKey.Value <-  s); Log.line "switched to %s" s
                | 'r' -> let s = "kaunertal"   in transact (fun _ -> curKey.Value <-  s); Log.line "switched to %s" s
                | 'e' -> let s = "himmelreich"   in transact (fun _ -> curKey.Value <-  s); Log.line "switched to %s" s
                | _ -> ()
    
    app.RenderTask <- app.Runtime.CompileRender(app.FramebufferSignature, sg)
    app.Run()
    0

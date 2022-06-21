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

let pois =
    [
        //"2020078-Eltville_store",1,V3d(3437123.4634,5543482.1291,91.4592),"Buero"
        "innen_store",1,V3d(-64.6625,-12.0023,349.9444),"asduh"
        "innen_store",2,V3d(-76.9321,-18.2107,349.9959),"aus"
    ]
    |> List.map (fun (pc,num,loc,name) -> (pc,num),(loc,name))
    |> Map.ofList

[<EntryPoint;STAThread>]
let main argv = 
    Aardvark.Init()

    use app = new Aardvark.Application.OpenVR.OpenGlVRApplicationLayered(4, DebugLevel.None)


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

    let f = 
        let oldState = ref Unchecked.defaultof<_> 
        let res =
            AdaptiveFunc<V3d>(fun res o ->
                let s = t.GetValue res
                let dt : TimeSpan = s - !oldState
                let v = velocity.GetValue res
                let newA = o + v * dt.TotalSeconds
                oldState := s 
                newA
            )
        oldState := t.GetValue()
        lock t (fun () ->
            t.Outputs.Add res |> ignore
        )

        AVal.constant res

        //velocity |> AVal.map (fun v ->
        //    if v = V3d.Zero then
        //        AdaptiveFunc.Identity
        //    else
        //        t |> AVal.stepTime (fun _ dt (off : V3d) ->
        //            dt.TotalSeconds * v + off
        //        )
        //)
    
    let t = AVal.integrate V3d.Zero app.Time [f]


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
                                    pointSize.Value <- clamp 0.5 20.0 (pointSize.Value + delta.Y)
                                    planeFitRadius.Value <- clamp 1.0 100.0 (planeFitRadius.Value + delta.X)
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
                            | 1u -> printfn "burger"
                            | 2u -> transact (fun () -> planeFit.Value <- not planeFit.Value)
                            | _ -> ()
                        | _ ->
                            ()
                    )
                    ()
                | _ ->
                    ()
        )


    let store = @"C:\Users\aszabo\bla\innen_store"
    let key = Path.combine [(if System.IO.Directory.Exists store then store else System.IO.Path.GetDirectoryName store);"key.txt"] |> File.readAllText
    let inst = Importy.myImport store (new LruDictionary<_,_>(1<<<20)) key |> Option.get
    let root = (inst.root.Id :?> IPointCloudNode)
    let centroid = V3d root.CentroidLocal + root.Cell.GetCenter()
    
    let s = cval 1.0
    
    let poi = cval centroid

    let centerTrafo = 
        poi |> AVal.map (fun poi -> 
            let d = poi - centroid


            (Trafo3d.Translation -d) *
            Trafo3d.Translation -centroid * 
            Trafo3d.Identity
        )

    let cfg = { LodTreeRenderConfig.simple with views = app.Info.viewTrafos; projs = app.Info.projTrafos }

    let getPlayerPos() =
        let fw = 
            Trafo3d.Translation(-centroid) *
            (Trafo3d.Translation(t.GetValue())) *
            (Trafo3d.Scale(s.GetValue()*V3d.III)) *
            app.Info.viewTrafos.GetValue().[0] *
            Trafo3d.Identity
        fw.Backward.C3.XYZ

    let myname = if Directory.Exists store then Path.GetFileName store else Path.GetFileName(System.IO.Path.GetDirectoryName store)
    let setPlayerPosToPoi (i : int) =
        match Map.tryFind (myname,i) pois with
        | Some(loc,name) -> 
            Log.line "Jump to %s,%d: %s" myname i name
            transact(fun _ -> poi.Value <- loc)
        | None -> Log.warn "No POI: %s, %d" myname i

    let reset() =
        Log.line "reset"
        transact (fun _ -> 
            poi.Value <- centroid
            s.Value <- 1.0
        )

    let vis =
        color |> AVal.map (function
            | true -> PointVisualization.Color
            | false -> PointVisualization.None
        )


    let sg = 
        Sg.lodTree cfg (ASet.single inst)
        |> Sg.shader {
            do! PointSetShaders.lodPointSize2
            do! PointSetShaders.lodPointCircular
        }
        |> Sg.trafo centerTrafo
        |> Sg.translation t
        |> Sg.scaling (s |> AVal.map ((*)V3d.III))
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


    let pass0 = render sg
    
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
    let sg =
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
                | 'm' -> printfn "\"%s\",%s" myname (getPlayerPos() |> (fun v -> sprintf "V3d(%.4f,%.4f,%.4f)" v.X v.Y v.Z))
                | 'l' -> reset()
                | '1' -> setPlayerPosToPoi 1
                | '2' -> setPlayerPosToPoi 2
                | '3' -> setPlayerPosToPoi 3
                | '4' -> setPlayerPosToPoi 4
                | '5' -> setPlayerPosToPoi 5
                | '6' -> setPlayerPosToPoi 6
                | '7' -> setPlayerPosToPoi 7
                | '8' -> setPlayerPosToPoi 8
                | '9' -> setPlayerPosToPoi 9
                | '0' -> setPlayerPosToPoi 0
                | _ -> ()
    
    app.RenderTask <- app.Runtime.CompileRender(app.FramebufferSignature, sg)
    app.Run()
    0

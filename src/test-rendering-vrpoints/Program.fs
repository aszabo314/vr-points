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

let pois =
    [
        "2020078-Eltville_store",1,V3d(3437123.4634,5543482.1291,91.4592),"Buero"
        "2020078-Eltville_store",2,V3d(3437130.9592,5543490.0154,96.9409),"Prunksaal"
    ]
    |> List.map (fun (pc,num,loc,name) -> (pc,num),(loc,name))
    |> Map.ofList

[<EntryPoint;STAThread>]
let main argv = 
    Aardvark.Init()

    let win = 
        window {
            display Display.OpenVR
            
        }
               
    //let sg =
    //    Sg.box' C4b.Red Box3d.Unit
    //    |> Sg.shader {
    //        do! DefaultSurfaces.trafo
    //        do! DefaultSurfaces.simpleLighting
    //    }

    let store = @"C:\Users\aszabo\bla\Eltville\2020078-Eltville_store"
    let key = Path.combine [(if System.IO.Directory.Exists store then store else System.IO.Path.GetDirectoryName store);"key.txt"] |> File.readAllText
    let inst = Importy.myImport store (new LruDictionary<_,_>(1<<<20)) key |> Option.get
    let root = (inst.root.Id :?> IPointCloudNode)
    let centroid = V3d root.CentroidLocal + root.Cell.GetCenter()
    
    let t = cval V3d.Zero
    let s = cval 1.0
    
    let poi = cval centroid

    let centerTrafo = 
        poi |> AVal.map (fun poi -> 
            let d = poi - centroid


            (Trafo3d.Translation -d) *
            Trafo3d.Translation -centroid * 
            Trafo3d.Identity
        )

    let cfg = { LodTreeRenderConfig.simple with views = win.View; projs = win.Proj }

    let getPlayerPos() =
        let fw = 
            Trafo3d.Translation(-centroid) *
            (Trafo3d.Translation(t.GetValue())) *
            (Trafo3d.Scale(s.GetValue()*V3d.III)) *
            win.View.GetValue().[0] *
            Trafo3d.Identity
        fw.Backward.C3.XYZ

    let setPlayerPosToPoi (i : int) =
        let myname = if Directory.Exists store then Path.GetFileName store else Path.GetFileName(System.IO.Path.GetDirectoryName store)
        match Map.tryFind (myname,i) pois with
        | Some(loc,name) -> 
            Log.line "Jump to %s,%d: %s" myname i name
            transact(fun _ -> poi.Value <- loc)
        | None -> Log.warn "No POI: %s, %d" myname i

    let reset() =
        Log.line "reset"
        transact (fun _ -> 
            poi.Value <- centroid
            t.Value <- V3d.Zero
            s.Value <- 1.0
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
        |> Sg.uniform "PointSize" (AVal.constant 5.0)
        |> Sg.uniform "ViewportSize" win.Sizes
        |> Sg.uniform "PointVisualization" (PointVisualization.Color |> AVal.constant)
        |> Sg.uniform "MagicExp" (~~0.0)
        |> Sg.uniform "ViewTrafo" win.View
        |> Sg.uniform "ProjTrafo" win.Proj

    let thread =
        startThread <| fun () ->
            while true do
                let l = Console.ReadKey()
                let v = win.View |> AVal.force
                let f = v.[0].Backward.C2.XYZ * V3d.IIO |> Vec.normalize
                let r = V3d(f.Y, -f.X, 0.0)
                let u = V3d.OOI
                match l.KeyChar with
                | 'd' -> transact (fun () -> t.Value <- t.Value + 0.5 * r)
                | 'a' -> transact (fun () -> t.Value <- t.Value - 0.5 * r)
                | 'w' -> transact (fun () -> t.Value <- t.Value + 0.5 * f)
                | 's' -> transact (fun () -> t.Value <- t.Value - 0.5 * f)
                | 'c' -> transact (fun () -> t.Value <- t.Value + 0.25 * u)
                | 'e' -> transact (fun () -> t.Value <- t.Value - 0.25 * u)
                | 't' -> transact (fun () -> s.Value <- s.Value + 0.02)
                | 'r' -> transact (fun () -> s.Value <- s.Value - 0.02)
                | 'm' -> printfn "%s" (getPlayerPos() |> (fun v -> sprintf "V3d(%.4f,%.4f,%.4f)" v.X v.Y v.Z))
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



    win.Scene <- sg
    win.Run()
    0
